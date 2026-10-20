// 2D polynomial least squares in normalised (distance, radial velocity).
// Basis terms are dn^i * rn^j ordered by total degree: 1, d, r, d^2, d*r, r^2, d^3, ...

/** Exponent pairs [i, j] for every term up to the given total degree. */
export function basisTerms(degree) {
  const terms = [];
  for (let k = 0; k <= degree; k++) {
    for (let i = k; i >= 0; i--) terms.push([i, k - i]);
  }
  return terms;
}

export function basisRow(terms, dn, rn) {
  return terms.map(([i, j]) => dn ** i * rn ** j);
}

export function normalize(norm, d, r) {
  return [(d - norm.dMid) / norm.dHalf, (r - norm.rMid) / norm.rHalf];
}

/** Evaluate a fitted polynomial at physical (d, r). */
export function evalPoly(terms, coeffs, norm, d, r) {
  const [dn, rn] = normalize(norm, d, r);
  let v = 0;
  for (let k = 0; k < terms.length; k++) v += coeffs[k] * dn ** terms[k][0] * rn ** terms[k][1];
  return v;
}

/** Solve A x = b (A square) with Gaussian elimination and partial pivoting. */
export function solve(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-14) throw new Error('singular fit matrix (too few valid grid points?)');
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

/**
 * Least-squares fit of values y at points (d, r).
 * @returns {{coefficients: number[], rms: number, maxAbs: number, maxOver: number}}
 *   maxOver = largest (fit - actual), used to make margin fits conservative.
 */
export function fitPoly(terms, norm, pts, y) {
  const n = terms.length;
  const AtA = Array.from({ length: n }, () => new Array(n).fill(0));
  const Atb = new Array(n).fill(0);
  const rows = pts.map(([d, r]) => {
    const [dn, rn] = normalize(norm, d, r);
    return basisRow(terms, dn, rn);
  });
  rows.forEach((row, k) => {
    for (let i = 0; i < n; i++) {
      Atb[i] += row[i] * y[k];
      for (let j = 0; j < n; j++) AtA[i][j] += row[i] * row[j];
    }
  });
  const coefficients = solve(AtA, Atb);
  let ss = 0;
  let maxAbs = 0;
  let maxOver = -Infinity;
  rows.forEach((row, k) => {
    const f = row.reduce((s, v, i) => s + v * coefficients[i], 0);
    const e = f - y[k];
    ss += e * e;
    maxAbs = Math.max(maxAbs, Math.abs(e));
    maxOver = Math.max(maxOver, e);
  });
  return { coefficients, rms: Math.sqrt(ss / Math.max(1, y.length)), maxAbs, maxOver };
}
