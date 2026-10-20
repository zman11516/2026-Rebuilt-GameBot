// Runtime evaluation of an exported shot map. Reference implementation for
// frc.robot.subsystems.shooter.shotmap.ShotMap (the Java parity tests compare against this).

/** Value of polynomial output `name` at (d, vr). */
export function polyValue(map, name, d, vr) {
  const n = map.normalization;
  const dn = (d - n.dMid) / n.dHalf;
  const rn = (vr - n.rMid) / n.rHalf;
  const c = map.coefficients[name];
  let v = 0;
  for (let k = 0; k < map.basis.length; k++) v += c[k] * dn ** map.basis[k][0] * rn ** map.basis[k][1];
  return v;
}

/** Index i with axis[i] <= v <= axis[i+1] (clamped) and the fraction between them. */
function locate(axis, v) {
  if (v <= axis[0]) return [0, 0];
  const last = axis.length - 1;
  if (v >= axis[last]) return [Math.max(0, last - 1), last === 0 ? 0 : 1];
  let i = 0;
  while (i < last - 1 && axis[i + 1] < v) i++;
  return [i, (v - axis[i]) / (axis[i + 1] - axis[i])];
}

/** Bilinear value of raw-grid output `name`; null when any of the four corners is invalid. */
export function gridValue(map, name, d, vr) {
  const g = map.rawGrid[name];
  const [di, fd] = locate(map.grid.d, d);
  const [ri, fr] = locate(map.grid.vr, vr);
  const di1 = Math.min(di + 1, map.grid.d.length - 1);
  const ri1 = Math.min(ri + 1, map.grid.vr.length - 1);
  const v00 = g[ri][di];
  const v01 = g[ri][di1];
  const v10 = g[ri1][di];
  const v11 = g[ri1][di1];
  if (v00 === null || v01 === null || v10 === null || v11 === null) return null;
  return (1 - fr) * ((1 - fd) * v00 + fd * v01) + fr * ((1 - fd) * v10 + fd * v11);
}

/** Valid distance range for a radial velocity: the intersection of the two bracketing rows. */
export function distanceRange(map, vr) {
  const rows = map.rows;
  if (vr < map.bounds.rMin || vr > map.bounds.rMax) return null;
  let i = 0;
  while (i < rows.length - 2 && rows[i + 1].vr < vr) i++;
  const a = rows[i];
  const b = rows[Math.min(i + 1, rows.length - 1)];
  if (a.dMin === null || b.dMin === null) return null;
  const dMin = Math.max(a.dMin, b.dMin);
  const dMax = Math.min(a.dMax, b.dMax);
  return dMin <= dMax ? { dMin, dMax } : null;
}

/**
 * Evaluate the map at (d, vr).
 * mode 'POLY' uses the fitted polynomials, 'GRID' bilinear raw-grid interpolation.
 * Returns { valid, reason, angleDeg, speed, tof, angleMarginLow, ..., vtMax } (values always
 * filled using clamped inputs so the turret can keep tracking even when the shot is invalid).
 */
export function evalMap(map, d, vr, mode = 'POLY') {
  let valid = true;
  let reason = 'OK';
  const range = distanceRange(map, vr);
  if (!range) {
    valid = false;
    reason = 'RADIAL_OUT_OF_RANGE';
  } else if (d < range.dMin || d > range.dMax) {
    valid = false;
    reason = 'DISTANCE_OUT_OF_RANGE';
  }
  const dc = Math.min(Math.max(d, range ? range.dMin : map.bounds.dMin), range ? range.dMax : map.bounds.dMax);
  const rc = Math.min(Math.max(vr, map.bounds.rMin), map.bounds.rMax);
  const out = { valid, reason };
  for (const name of map.outputs) {
    let v = mode === 'GRID' ? gridValue(map, name, dc, rc) : polyValue(map, name, dc, rc);
    if (v === null) {
      v = polyValue(map, name, dc, rc);
      if (out.valid) {
        out.valid = false;
        out.reason = 'GRID_CELL_INVALID';
      }
    }
    out[name] = name.includes('Margin') ? Math.max(0, v) : v;
  }
  const vt = gridValue(map, 'vtMax', dc, rc);
  out.vtMax = vt === null ? map.tangentialLimit.global : vt;
  return out;
}
