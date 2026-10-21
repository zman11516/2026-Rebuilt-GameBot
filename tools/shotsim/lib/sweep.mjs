// Per-angle speed search: for each launch angle find [sLow, sHigh], the exit speeds that score.

/**
 * Smallest s in [lo, hi] with f(s) >= 0 for an increasing f, by bisection.
 * Returns Infinity when f(hi) < 0, lo when f(lo) >= 0.
 */
function firstNonNegative(f, lo, hi, tol) {
  if (f(hi) < 0) return Infinity;
  if (f(lo) >= 0) return lo;
  while (hi - lo > tol) {
    const mid = 0.5 * (lo + hi);
    if (f(mid) >= 0) hi = mid;
    else lo = mid;
  }
  return hi;
}

/** Largest s in [lo, hi] with f(s) >= 0 for a decreasing f. Returns -Infinity when f(lo) < 0. */
function lastNonNegative(f, lo, hi, tol) {
  if (f(lo) < 0) return -Infinity;
  if (f(hi) >= 0) return hi;
  while (hi - lo > tol) {
    const mid = 0.5 * (lo + hi);
    if (f(mid) >= 0) lo = mid;
    else hi = mid;
  }
  return lo;
}

/**
 * Bracket a root near a previous answer before bisecting the full range (warm start between
 * neighbouring angles cuts the number of simulations roughly in half).
 */
function warmBracket(f, guess, lo, hi, step, increasing) {
  if (!Number.isFinite(guess)) return [lo, hi];
  let a = Math.max(lo, guess - step);
  let b = Math.min(hi, guess + step);
  for (let i = 0; i < 6; i++) {
    const fa = f(a);
    const fb = f(b);
    const okA = increasing ? fa < 0 : fa >= 0;
    const okB = increasing ? fb >= 0 : fb < 0;
    if (okA && okB) return [a, b];
    if (!okA) a = Math.max(lo, a - step * 2 ** (i + 1));
    if (!okB) b = Math.min(hi, b + step * 2 ** (i + 1));
    if (a === lo && b === hi) break;
  }
  return [lo, hi];
}

/**
 * Sweep launch angles.
 * @param {(angleDeg: number, s: number) => {gLow: number, gHigh: number, tof: number, apex: number}} shot
 * @param {number[]} angles launch angles (deg), ascending
 * @param {{min: number, max: number}} speed search range (m/s)
 * @param {number} tol speed tolerance (m/s)
 * @returns {{angle: number, sLow: number, sHigh: number, valid: boolean}[]}
 */
export function sweepAngles(shot, angles, speed, tol = 0.005) {
  const out = [];
  let prevLow = NaN;
  let prevHigh = NaN;
  for (const angle of angles) {
    const cache = new Map();
    const sim = (s) => {
      let r = cache.get(s);
      if (!r) {
        r = shot(angle, s);
        cache.set(s, r);
      }
      return r;
    };
    const gLow = (s) => sim(s).gLow;
    const gHigh = (s) => sim(s).gHigh;
    const [l0, l1] = warmBracket(gLow, prevLow, speed.min, speed.max, 0.4, true);
    const sLow = firstNonNegative(gLow, l0, l1, tol);
    const [h0, h1] = warmBracket(gHigh, prevHigh, speed.min, speed.max, 0.4, false);
    const sHigh = lastNonNegative(gHigh, h0, h1, tol);
    const valid = Number.isFinite(sLow) && Number.isFinite(sHigh) && sLow <= sHigh;
    out.push({ angle, sLow, sHigh, valid });
    prevLow = Number.isFinite(sLow) ? sLow : NaN;
    prevHigh = Number.isFinite(sHigh) ? sHigh : NaN;
  }
  return out;
}

/**
 * Angles from min to max, padded on both sides. The step is adjusted (to the nearest whole number of
 * steps across [min, max]) so both ends land exactly on the list: the hood's stops are always candidates.
 */
export function angleList(min, max, step, pad = 0) {
  const n = max > min ? Math.max(1, Math.round((max - min) / step)) : 0;
  const s = n > 0 ? (max - min) / n : step;
  const k = Math.ceil(pad / s - 1e-9);
  const a = [];
  for (let i = -k; i <= n + k; i++) a.push(i === n ? max : i === 0 ? min : Math.round((min + i * s) * 1e9) / 1e9);
  return a;
}
