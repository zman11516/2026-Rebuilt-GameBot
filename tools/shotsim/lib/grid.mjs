// Full shot map: sweep + select at every (distance, radial velocity) grid point.

import { makeAero } from './physics.mjs';
import { angleForHoodRot, angleRange, speedRange } from './mechanism.mjs';
import { simulateHub, simulateFeed, feedProfile } from './targets.mjs';
import { sweepAngles, angleList } from './sweep.mjs';
import { selectMaximin } from './select.mjs';

const DEG = Math.PI / 180;

export function axis(spec) {
  const n = Math.round((spec.max - spec.min) / spec.step);
  return Array.from({ length: n + 1 }, (_, i) => Math.round((spec.min + i * spec.step) * 1e6) / 1e6);
}

/**
 * Everything needed to evaluate one shot for a config: returns shot(d, vr, profile) => (angle, s) => result.
 */
export function makeContext(config, mech) {
  const aero = makeAero(config.physics);
  const range = angleRange(mech);
  const fixedAngle =
    config.selection.fixedHoodRot !== null && config.selection.fixedHoodRot !== undefined
      ? angleForHoodRot(mech, config.selection.fixedHoodRot)
      : null;
  const angles =
    fixedAngle !== null
      ? angleList(fixedAngle, fixedAngle, config.selection.angleStepDeg, config.selection.anglePadDeg)
      : angleList(range.min, range.max, config.selection.angleStepDeg, config.selection.anglePadDeg);
  const speed = speedRange(mech);
  const shotFor = (d, vr, profile) => {
    if (config.kind === 'hub') return (angle, s) => simulateHub(aero, mech, config.hub, d, vr, angle, s);
    return (angle, s) => simulateFeed(aero, mech, config.feed, d, vr, angle, s, profile.intervals);
  };
  return { aero, range, fixedAngle, angles, speed, shotFor };
}

/** Largest tangential speed the turret can cancel at this shot (m/s). */
export function tangentialLimit(config, mech, range, speedMax, d, angleDeg, speed, tof) {
  const H = speed * Math.cos(angleDeg * DEG);
  const V = speed * Math.sin(angleDeg * DEG);
  // Cancelling vt tilts the shooter vector sideways, which lowers its elevation and raises its speed.
  const hRoom = V / Math.tan(range.min * DEG);
  const vtHood = hRoom > H ? Math.sqrt(hRoom * hRoom - H * H) : 0;
  const sRoom = speedMax * speedMax - H * H - V * V;
  const vtSpeed = sRoom > 0 ? Math.sqrt(sRoom) : 0;
  let vt = Math.min(config.tangential.absMax, vtHood, vtSpeed);
  const rate = config.tangential.maxBearingRateRadPerSec;
  if (rate && tof > 0) vt = Math.min(vt, (d * Math.tan(rate * tof)) / tof);
  return vt;
}

/** Solve one grid point. Returns the selected shot and the bands used to pick it. */
export function solvePoint(ctx, config, mech, d, vr, profile) {
  if (config.kind !== 'hub' && (!profile || profile.bearings.length === 0)) {
    return { d, vr, valid: false, reason: 'NO_LAUNCH_BEARING', bands: [] };
  }
  const shot = ctx.shotFor(d, vr, profile);
  const bands = sweepAngles(shot, ctx.angles, ctx.speed, config.selection.speedTol);
  const best = selectMaximin(bands, {
    sigmaSpeed: config.selection.sigmaSpeed,
    sigmaAngleDeg: config.selection.sigmaAngleDeg,
    angleMin: ctx.range.min,
    angleMax: ctx.range.max,
    speedMax: ctx.speed.max,
    speedBias: config.selection.speedBias,
    fixedAngle: ctx.fixedAngle ?? undefined,
  });
  if (!best) {
    // A launch point hugging an obstacle it cannot clear (e.g. at the trench mouth) must not wipe
    // out the whole distance: drop those bearings if they cannot score alone and solve the rest.
    // A bearing blocked by an obstacle further along its line still counts against the distance.
    const only = (bs) => ({ bearings: bs, intervals: profile.intervals.filter((i) => bs.includes(i.bearingDeg)) });
    const hug = config.feed?.hugDistance ?? 0;
    const dropped =
      config.kind === 'hub' || profile.bearings.length < 2
        ? []
        : profile.bearings.filter(
            (b) => only([b]).intervals.some((i) => i.x1 < hug) && !solvePoint(ctx, config, mech, d, vr, only([b])).valid,
          );
    if (dropped.length > 0 && dropped.length < profile.bearings.length) {
      const p = solvePoint(ctx, config, mech, d, vr, only(profile.bearings.filter((b) => !dropped.includes(b))));
      return { ...p, droppedBearings: dropped };
    }
    return { d, vr, valid: false, reason: 'NO_SCORING_SHOT', bands };
  }
  const r = shot(best.angle, best.speed);
  const vtMax = tangentialLimit(config, mech, ctx.range, ctx.speed.max, d, best.angle, best.speed, r.tof);
  return {
    d,
    vr,
    valid: true,
    reason: 'OK',
    angleDeg: best.angle,
    speed: best.speed,
    tof: r.tof,
    apex: r.apex,
    angleMarginLow: best.angleMarginLow,
    angleMarginHigh: best.angleMarginHigh,
    speedMarginLow: best.speedMarginLow,
    speedMarginHigh: best.speedMarginHigh,
    score: best.score,
    vtMax,
    bands,
  };
}

/**
 * Build the whole grid. rowFilter (optional) limits which radial-velocity rows are solved so a
 * browser can split the work across Web Workers.
 */
export function buildGrid(config, mech, { onProgress, rowFilter, keepBands = false } = {}) {
  const ctx = makeContext(config, mech);
  const dValues = axis(config.grid.distance);
  const rValues = axis(config.grid.radialVelocity);
  const target = config.kind === 'hub' ? null : config.target;
  const profiles = new Map();
  const profileFor = (d) => {
    if (config.kind === 'hub') return null;
    if (!profiles.has(d)) profiles.set(d, feedProfile(config.feed, target, d, ctx.aero.r));
    return profiles.get(d);
  };
  const points = [];
  const total = dValues.length * rValues.length;
  let done = 0;
  rValues.forEach((vr, ri) => {
    if (rowFilter && !rowFilter(ri)) return;
    dValues.forEach((d, di) => {
      const p = solvePoint(ctx, config, mech, d, vr, profileFor(d));
      if (!keepBands) delete p.bands;
      p.ri = ri;
      p.di = di;
      points.push(p);
      done++;
      if (onProgress) onProgress(done, total);
    });
  });
  return { dValues, rValues, points, angleRange: ctx.range, speedRange: ctx.speed };
}
