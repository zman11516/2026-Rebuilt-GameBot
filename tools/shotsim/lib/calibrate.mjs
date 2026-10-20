// Fit the two calibration knobs (hood angle at CANcoder 0, exit speed per rotor rps) to shots
// the team already tuned by hand: rows of (distance, flywheel rps, hood rotations[, tof]).
// Each tuned shot went in, so the model should put its entry-plane crossing near the hub centre.

import { makeAero } from './physics.mjs';
import { simulateHub } from './targets.mjs';
import { angleForHoodRot } from './mechanism.mjs';

/** Parse "d,rps,hood[,tof]" CSV text (header line and # comments allowed). */
export function parseTunedCsv(text) {
  return text
    .split(/\r?\n/)
    .map((l) => l.trim())
    .filter((l) => l && !l.startsWith('#') && !/^[a-z]/i.test(l))
    .map((l) => {
      const [d, rps, hoodRot, tof] = l.split(',').map(Number);
      return { d, rps, hoodRot, tof: Number.isFinite(tof) ? tof : null };
    });
}

function withParams(mech, theta0Deg, kv) {
  return {
    ...mech,
    hood: { ...mech.hood, theta0Deg },
    flywheel: { ...mech.flywheel, exitSpeedPerRotorRps: kv, exitSpeedScale: 1 },
  };
}

/** Model prediction for every tuned row with the given knobs. */
export function predictRows(hubConfig, mech, rows, theta0Deg, kv) {
  const aero = makeAero(hubConfig.physics);
  const m = withParams(mech, theta0Deg, kv);
  const w = hubConfig.hub.halfWidth - aero.r - hubConfig.hub.windowInset;
  return rows.map((row) => {
    const angle = angleForHoodRot(m, row.hoodRot);
    const speed = kv * row.rps;
    const r = simulateHub(aero, m, hubConfig.hub, row.d, 0, angle, speed);
    return {
      ...row,
      angleDeg: angle,
      speed,
      xCross: r.xc,
      miss: r.xc - row.d,
      tofModel: r.tof,
      scores: r.ok,
      reason: r.reason,
      windowHalfWidth: w,
    };
  });
}

function cost(pred, { sigmaX, sigmaTof, useTof }) {
  let c = 0;
  for (const p of pred) {
    if (!Number.isFinite(p.xCross)) {
      c += 1e4;
      continue;
    }
    c += (p.miss / sigmaX) ** 2;
    if (useTof && p.tof !== null && Number.isFinite(p.tofModel)) c += ((p.tofModel - p.tof) / sigmaTof) ** 2;
  }
  return c;
}

/** 2D Nelder-Mead. */
function nelderMead(f, x0, step, iters = 200) {
  let simplex = [x0, [x0[0] + step[0], x0[1]], [x0[0], x0[1] + step[1]]].map((x) => ({ x, f: f(x) }));
  for (let k = 0; k < iters; k++) {
    simplex.sort((a, b) => a.f - b.f);
    const [best, mid, worst] = simplex;
    const c = [(best.x[0] + mid.x[0]) / 2, (best.x[1] + mid.x[1]) / 2];
    const pt = (t) => [c[0] + t * (worst.x[0] - c[0]), c[1] + t * (worst.x[1] - c[1])];
    const r = { x: pt(-1), f: 0 };
    r.f = f(r.x);
    if (r.f < best.f) {
      const e = { x: pt(-2), f: 0 };
      e.f = f(e.x);
      simplex[2] = e.f < r.f ? e : r;
    } else if (r.f < mid.f) {
      simplex[2] = r;
    } else {
      const ct = { x: pt(0.5), f: 0 };
      ct.f = f(ct.x);
      if (ct.f < worst.f) simplex[2] = ct;
      else simplex = simplex.map((s, i) => (i === 0 ? s : { x: [(s.x[0] + best.x[0]) / 2, (s.x[1] + best.x[1]) / 2], f: 0 })).map((s, i) => (i === 0 ? s : { x: s.x, f: f(s.x) }));
    }
    const spread = Math.abs(simplex[0].f - simplex[2].f);
    if (spread < 1e-9 && k > 20) break;
  }
  simplex.sort((a, b) => a.f - b.f);
  return simplex[0];
}

/**
 * Fit theta0 (deg) and kv (m/s per rotor rps).
 * opts: { sigmaX = 0.1, sigmaTof = 0.08, useTof = false, theta0Guess, kvGuess }
 */
export function calibrate(hubConfig, mech, rows, opts = {}) {
  const o = { sigmaX: 0.1, sigmaTof: 0.08, useTof: false, ...opts };
  const f = ([t0, kv]) => cost(predictRows(hubConfig, mech, rows, t0, kv), o);
  // Coarse scan first: the cost is flat along a valley, so start near its floor.
  let start = null;
  for (let t0 = 55; t0 <= 80; t0 += 1) {
    for (let kv = 0.11; kv <= 0.19; kv += 0.005) {
      const c = f([t0, kv]);
      if (!start || c < start.f) start = { x: [t0, kv], f: c };
    }
  }
  const best = nelderMead(f, opts.theta0Guess !== undefined ? [opts.theta0Guess, opts.kvGuess] : start.x, [1, 0.004]);
  const [theta0Deg, exitSpeedPerRotorRps] = best.x;
  const profile = [];
  for (let t0 = Math.round(theta0Deg) - 8; t0 <= Math.round(theta0Deg) + 6; t0 += 2) {
    // best kv for this theta0 (1D golden-free scan is plenty here)
    let b = Infinity;
    for (let kv = 0.11; kv <= 0.19; kv += 0.002) b = Math.min(b, f([t0, kv]));
    profile.push({ theta0Deg: t0, cost: b });
  }
  return {
    theta0Deg,
    exitSpeedPerRotorRps,
    cost: best.f,
    rows: predictRows(hubConfig, mech, rows, theta0Deg, exitSpeedPerRotorRps),
    profile,
  };
}
