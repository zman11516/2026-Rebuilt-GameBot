// Shot classification against the HUB (scoring) and a FEED landing zone (passing).
//
// Each evaluator simulates one shot and returns two margins that make the speed search a pair
// of 1D root finds:
//   gLow  >= 0  means "not short"  (increases with exit speed)
//   gHigh >= 0  means "not long"   (decreases with exit speed)
// A shot is good when both are >= 0, so at a fixed launch angle the good speeds form one
// interval [sLow, sHigh] (4414's red lower limit and green upper limit).

import { integrate, spinFor } from './physics.mjs';
import { releasePoint } from './mechanism.mjs';

const DEG = Math.PI / 180;

/** Distance from point (px, pz) to segment (ax, az)-(bx, bz). */
function segDist(px, pz, ax, az, bx, bz) {
  const dx = bx - ax;
  const dz = bz - az;
  const len2 = dx * dx + dz * dz;
  let u = len2 > 0 ? ((px - ax) * dx + (pz - az) * dz) / len2 : 0;
  if (u < 0) u = 0;
  else if (u > 1) u = 1;
  const qx = ax + u * dx - px;
  const qz = az + u * dz - pz;
  return Math.sqrt(qx * qx + qz * qz);
}

/** Initial conditions for a shot: release point, field velocity, spin. */
export function launchState(aero, mech, angleDeg, shooterSpeed, radialVelocity) {
  const rel = releasePoint(mech, angleDeg);
  const th = angleDeg * DEG;
  return {
    x0: rel.x,
    z0: rel.z,
    vx0: shooterSpeed * Math.cos(th) + radialVelocity,
    vz0: shooterSpeed * Math.sin(th),
    spin: spinFor(aero, shooterSpeed),
  };
}

/**
 * Simulate a hub shot from horizontal distance d (turret axis to hub centre).
 * Hub model (1D along the shot line, hub centre at x = d):
 *  - entry plane z = entryHeight; ball centre must cross it descending inside
 *    |x - d| <= halfWidth - r - windowInset
 *  - front face (vertical) at x = d - faceOffset, deck from the face to the opening's front
 *    edge at x = d - halfWidth; the ball centre must stay r + lipClearance away from both
 *  - apex <= apexMax
 */
export function simulateHub(aero, mech, hub, d, vr, angleDeg, s) {
  const L = launchState(aero, mech, angleDeg, s, vr);
  const he = hub.entryHeight;
  const w = hub.halfWidth - aero.r - hub.windowInset;
  const xf = d - hub.faceOffset;
  const xl = d - hub.halfWidth;
  const need = aero.r + hub.lipClearance;
  const lipLo = xf - need - 0.05;
  const lipHi = xl + need + 0.05;
  let apex = L.z0;
  let lipMin = Infinity;
  let xc = NaN;
  let tof = NaN;
  let vzc = NaN;
  integrate(aero, L.x0, L.z0, L.vx0, L.vz0, L.spin, (st) => {
    if (st.z > apex) apex = st.z;
    if (st.x >= lipLo && st.x <= lipHi) {
      const dFace = segDist(st.x, st.z, xf, 0, xf, he);
      const dDeck = segDist(st.x, st.z, xf, he, xl, he);
      const m = Math.min(dFace, dDeck) - need;
      if (m < lipMin) lipMin = m;
    }
    if (st.pz >= he && st.z < he && st.vz < 0) {
      const f = (st.pz - he) / (st.pz - st.z);
      xc = st.px + f * (st.x - st.px);
      tof = st.pt + f * (st.t - st.pt);
      vzc = st.pvz + f * (st.vz - st.pvz);
      return true;
    }
    // Fell back to the floor without ever crossing the entry plane, or flew far past the hub.
    return st.z < 0 || st.x > d + 3;
  });
  if (Number.isNaN(xc)) {
    // Never came down through the entry plane: short if it never got that high, otherwise it
    // sailed over the hub (still climbing past it, or timed out above it).
    if (apex < he) return { ok: false, gLow: -Infinity, gHigh: Infinity, apex, xc, tof, lipMin, reason: 'SHORT' };
    return { ok: false, gLow: Infinity, gHigh: -Infinity, apex, xc, tof, lipMin, reason: 'LONG' };
  }
  const gLow = Math.min(xc - (d - w), lipMin);
  const gHigh = Math.min(d + w - xc, hub.apexMax - apex);
  let reason = 'SCORE';
  if (lipMin < 0) reason = 'LIP';
  else if (xc < d - w) reason = 'SHORT';
  else if (apex > hub.apexMax) reason = 'APEX';
  else if (xc > d + w) reason = 'LONG';
  return { ok: gLow >= 0 && gHigh >= 0, gLow, gHigh, apex, xc, tof, lipMin, descentDeg: NaN, vzc, reason };
}

/**
 * Simulate a feed (pass) shot to a landing zone centred at x = d.
 * profile: array of { x1, x2, zReq } obstacle constraints along the shot line, where the ball
 * centre must stay at or above zReq while x is in [x1, x2].
 */
export function simulateFeed(aero, mech, feed, d, vr, angleDeg, s, profile) {
  const L = launchState(aero, mech, angleDeg, s, vr);
  const zl = feed.landingHeight;
  const R = feed.zoneRadius;
  let apex = L.z0;
  let obsMin = Infinity;
  let xland = NaN;
  let tof = NaN;
  integrate(aero, L.x0, L.z0, L.vx0, L.vz0, L.spin, (st) => {
    if (st.z > apex) apex = st.z;
    for (let i = 0; i < profile.length; i++) {
      const o = profile[i];
      if (st.x >= o.x1 && st.x <= o.x2) {
        const m = st.z - o.zReq;
        if (m < obsMin) obsMin = m;
      }
    }
    if (st.pz >= zl && st.z < zl && st.vz < 0) {
      const f = (st.pz - zl) / (st.pz - st.z);
      xland = st.px + f * (st.x - st.px);
      tof = st.pt + f * (st.t - st.pt);
      return true;
    }
    return st.x > d + R + 5;
  });
  if (Number.isNaN(xland)) {
    // Still in the air far past the zone (or out of time): long.
    return { ok: false, gLow: Infinity, gHigh: -Infinity, apex, xc: xland, tof, obsMin, reason: 'LONG' };
  }
  const gLow = Math.min(xland - (d - R), obsMin);
  const gHigh = Math.min(d + R - xland, feed.apexMax - apex);
  let reason = 'SCORE';
  if (obsMin < 0) reason = 'OBSTACLE';
  else if (xland < d - R) reason = 'SHORT';
  else if (apex > feed.apexMax) reason = 'APEX';
  else if (xland > d + R) reason = 'LONG';
  return { ok: gLow >= 0 && gHigh >= 0, gLow, gHigh, apex, xc: xland, tof, obsMin, reason };
}

/** Segment-vs-rectangle intersection in the field plane; returns [u1, u2] along the segment or null. */
function segRect(ax, ay, bx, by, x1, x2, y1, y2) {
  let t0 = 0;
  let t1 = 1;
  const dx = bx - ax;
  const dy = by - ay;
  const clip = (p, q) => {
    if (Math.abs(p) < 1e-12) return q >= 0;
    const r = q / p;
    if (p < 0) {
      if (r > t1) return false;
      if (r > t0) t0 = r;
    } else {
      if (r < t0) return false;
      if (r < t1) t1 = r;
    }
    return true;
  };
  if (clip(-dx, ax - x1) && clip(dx, x2 - ax) && clip(-dy, ay - y1) && clip(dy, y2 - ay)) return [t0, t1];
  return null;
}

function inBox(x, y, b) {
  return x >= b.x[0] && x <= b.x[1] && y >= b.y[0] && y <= b.y[1];
}

/** Obstacles as boxes {x:[..], y:[..], zTop}; vertical planes become zero-thickness boxes. */
export function obstacleBoxes(feed) {
  return feed.obstacles.map((o) => {
    if (o.type === 'plane') {
      const x = Array.isArray(o.x) ? o.x : [o.x, o.x];
      const y = Array.isArray(o.y) ? o.y : [o.y, o.y];
      return { name: o.name, x, y, zTop: o.z[1] };
    }
    return { name: o.name, x: o.x, y: o.y, zTop: o.zTop };
  });
}

/** True when the segment from (ax, ay) to (bx, by) passes over box b. */
export function crossesBox(ax, ay, bx, by, b) {
  return segRect(ax, ay, bx, by, b.x[0], b.x[1], b.y[0], b.y[1]) !== null;
}

/**
 * Worst-case obstacle profile for passing from distance d to a target.
 * Launch points are sampled on the circle of radius d around the target at several bearings
 * inside the allowed launch region; the trajectory in the shot plane is the same for every
 * bearing, so the union of every bearing's obstacle intervals is the constraint set.
 *
 * feed.crossing selects which bearings a map covers (4414 keeps "feed" and "net feed" apart):
 *  - { mode: 'avoid', obstacles: [...] }   only bearings whose line misses those obstacles
 *  - { mode: 'require', obstacles: [...] } only bearings whose line crosses one of them
 */
export function feedProfile(feed, target, d, r) {
  const boxes = obstacleBoxes(feed);
  const reg = feed.launchRegion;
  const need = r + feed.obstacleClearance;
  const crossNames = new Set(feed.crossing?.obstacles ?? []);
  const crossBoxes = boxes.filter((o) => crossNames.has(o.name));
  const bearings = [];
  const intervals = [];
  const n = feed.bearingSamples;
  for (let k = 0; k < n; k++) {
    const b = (-feed.bearingSpanDeg + (2 * feed.bearingSpanDeg * k) / Math.max(1, n - 1)) * DEG;
    const lx = target[0] + d * Math.cos(b);
    const ly = target[1] + d * Math.sin(b);
    if (lx < reg.x[0] || lx > reg.x[1] || ly < reg.y[0] || ly > reg.y[1]) continue;
    if (boxes.some((o) => inBox(lx, ly, o))) continue;
    if ((feed.noShootZones ?? []).some((z) => inBox(lx, ly, z))) continue;
    if (crossBoxes.length > 0) {
      const crosses = crossBoxes.some((o) => crossesBox(lx, ly, target[0], target[1], o));
      if (feed.crossing.mode === 'avoid' && crosses) continue;
      if (feed.crossing.mode === 'require' && !crosses) continue;
    }
    bearings.push(b / DEG);
    for (const o of boxes) {
      // Expand the footprint by the ball radius so the ball's edge, not its centre, must clear.
      const hit = segRect(lx, ly, target[0], target[1], o.x[0] - r, o.x[1] + r, o.y[0] - r, o.y[1] + r);
      if (hit) intervals.push({ name: o.name, bearingDeg: b / DEG, x1: hit[0] * d, x2: hit[1] * d, zReq: o.zTop + need });
    }
  }
  return { bearings, intervals };
}
