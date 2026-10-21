// Robust shot selection.
//
// maximin (4414's rule): for each launch angle, aim at the middle of the scoring speed window;
// the speed margin is half the window and the angle margin is how far the angle can drift
// (holding that speed) before the shot stops scoring. Normalise each margin by its expected
// error (sigma) and pick the shot whose WEAKEST normalised margin is largest.

/** Where s* leaves the scoring band between a contained angle a and an excluded angle b. */
function edgeAngle(a, b, s) {
  if (!b.valid) return 0.5 * (a.angle + b.angle);
  // Linear interpolation of whichever band edge s* crosses.
  const cross = (ea, eb) => {
    if (ea === eb) return 0.5;
    const f = (s - ea) / (eb - ea);
    return Math.min(Math.max(f, 0), 1);
  };
  let f = 1;
  if (s < b.sLow) f = Math.min(f, cross(a.sLow, b.sLow));
  if (s > b.sHigh) f = Math.min(f, cross(a.sHigh, b.sHigh));
  return a.angle + f * (b.angle - a.angle);
}

/** Angle margins (deg below, deg above) for holding speed s at band index i. */
export function angleMargins(bands, i, s) {
  const inside = (k) => k >= 0 && k < bands.length && bands[k].valid && bands[k].sLow <= s && s <= bands[k].sHigh;
  let lo = i;
  while (inside(lo - 1)) lo--;
  let hi = i;
  while (inside(hi + 1)) hi++;
  const loEdge = lo - 1 >= 0 ? edgeAngle(bands[lo], bands[lo - 1], s) : bands[lo].angle;
  const hiEdge = hi + 1 < bands.length ? edgeAngle(bands[hi], bands[hi + 1], s) : bands[hi].angle;
  return { below: bands[i].angle - loEdge, above: hiEdge - bands[i].angle };
}

/**
 * Pick the most robust shot from swept bands.
 * @param bands output of sweepAngles (ascending angle; may extend past the hood range so margins
 *   near the hood limits see the physics, not the mechanism)
 * @param opts { sigmaSpeed, sigmaAngleDeg, angleMin, angleMax, speedMax, speedBias, fixedAngle }
 */
export function selectMaximin(bands, opts) {
  let best = null;
  for (let i = 0; i < bands.length; i++) {
    const b = bands[i];
    if (!b.valid) continue;
    if (opts.fixedAngle !== undefined && opts.fixedAngle !== null) {
      if (Math.abs(b.angle - opts.fixedAngle) > 1e-6) continue;
    } else if (b.angle < opts.angleMin - 1e-9 || b.angle > opts.angleMax + 1e-9) continue;
    let s = 0.5 * (b.sLow + b.sHigh);
    if (opts.speedMax !== undefined && s > opts.speedMax) {
      if (b.sLow > opts.speedMax) continue;
      s = opts.speedMax;
    }
    const speedBelow = s - b.sLow;
    const speedAbove = b.sHigh - s;
    const am = angleMargins(bands, i, s);
    const score =
      Math.min(Math.min(speedBelow, speedAbove) / opts.sigmaSpeed, Math.min(am.below, am.above) / opts.sigmaAngleDeg) -
      (opts.speedBias ?? 0) * (s / opts.sigmaSpeed);
    if (!best || score > best.score + 1e-12 || (Math.abs(score - best.score) <= 1e-12 && s < best.speed)) {
      best = {
        angle: b.angle,
        speed: s,
        score,
        speedMarginLow: speedBelow,
        speedMarginHigh: speedAbove,
        angleMarginLow: am.below,
        angleMarginHigh: am.above,
      };
    }
  }
  return best;
}

/**
 * Low-power rule (for passes): the slowest shot whose four margins all stay at least
 * minMarginSigma sigmas. Speed sits that far above the window's lower edge instead of mid-window,
 * so the flywheel runs as slowly as the accuracy floor allows.
 * @param opts as selectMaximin, plus minMarginSigma (k)
 * @returns the same shape as selectMaximin, or null when no angle meets the floor
 */
export function selectMinPower(bands, opts) {
  const k = opts.minMarginSigma;
  let best = null;
  for (let i = 0; i < bands.length; i++) {
    const b = bands[i];
    if (!b.valid) continue;
    if (opts.fixedAngle !== undefined && opts.fixedAngle !== null) {
      if (Math.abs(b.angle - opts.fixedAngle) > 1e-6) continue;
    } else if (b.angle < opts.angleMin - 1e-9 || b.angle > opts.angleMax + 1e-9) continue;
    const s = b.sLow + k * opts.sigmaSpeed;
    if (s > b.sHigh - k * opts.sigmaSpeed + 1e-12) continue;
    if (opts.speedMax !== undefined && s > opts.speedMax) continue;
    const am = angleMargins(bands, i, s);
    if (Math.min(am.below, am.above) < k * opts.sigmaAngleDeg - 1e-12) continue;
    const score = Math.min(Math.min(s - b.sLow, b.sHigh - s) / opts.sigmaSpeed, Math.min(am.below, am.above) / opts.sigmaAngleDeg);
    if (!best || s < best.speed - 1e-12 || (Math.abs(s - best.speed) <= 1e-12 && score > best.score)) {
      best = {
        angle: b.angle,
        speed: s,
        score,
        speedMarginLow: s - b.sLow,
        speedMarginHigh: b.sHigh - s,
        angleMarginLow: am.below,
        angleMarginHigh: am.above,
      };
    }
  }
  return best;
}

/**
 * Selection by mode: 'maximin' (most robust, used for the hub) or 'minPower' (slowest shot with
 * every margin >= minMarginSigma, used for passes). minPower falls back to maximin where no shot
 * meets the floor and marks the result fallback: true.
 */
export function selectShot(bands, mode, opts) {
  if (mode === 'minPower') {
    const low = selectMinPower(bands, opts);
    if (low) return { ...low, selection: 'minPower' };
    const fb = selectMaximin(bands, opts);
    return fb ? { ...fb, selection: 'maximin-fallback', fallback: true } : null;
  }
  const best = selectMaximin(bands, opts);
  return best ? { ...best, selection: 'maximin' } : null;
}

/** Small seeded PRNG (mulberry32) so Monte Carlo readouts are repeatable. */
export function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Standard normal sample (Box-Muller). */
function gauss(rand) {
  const u = Math.max(rand(), 1e-12);
  const v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/**
 * Monte Carlo cross-check: fraction of shots that still score when angle and speed carry
 * Gaussian errors with the selection sigmas.
 */
export function probabilityOfScore(shot, angle, speed, { sigmaSpeed, sigmaAngleDeg, samples = 256, seed = 2026 }) {
  const rand = rng(seed);
  let hits = 0;
  for (let i = 0; i < samples; i++) {
    const r = shot(angle + sigmaAngleDeg * gauss(rand), speed + sigmaSpeed * gauss(rand));
    if (r.gLow >= 0 && r.gHigh >= 0) hits++;
  }
  return hits / samples;
}
