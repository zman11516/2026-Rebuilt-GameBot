// Shot-map JSON export (schema "shotsim5454/1"), read on the robot by
// frc.robot.subsystems.shooter.shotmap.ShotMap.

import { basisTerms, fitPoly } from './fit.mjs';
import { crossingBoxes } from './targets.mjs';

export const SCHEMA = 'shotsim5454/1';
export const OUTPUTS = [
  'angleDeg',
  'speed',
  'tof',
  'angleMarginLow',
  'angleMarginHigh',
  'speedMarginLow',
  'speedMarginHigh',
];
const MARGINS = new Set(['angleMarginLow', 'angleMarginHigh', 'speedMarginLow', 'speedMarginHigh']);

/** Round for stable, diff-friendly JSON. */
export function round(v, digits = 6) {
  if (v === null || v === undefined || !Number.isFinite(v)) return null;
  const f = 10 ** digits;
  const r = Math.round(v * f) / f;
  return Object.is(r, -0) ? 0 : r;
}

/** FNV-1a hash of a string, hex (identifies the config a map was built from). */
export function hash(str) {
  let h = 0x811c9dc5;
  for (let i = 0; i < str.length; i++) {
    h ^= str.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, '0');
}

/** Per-row [vr, dMin, dMax] of valid distances (first and last valid column). */
export function rowRanges(grid) {
  return grid.rValues.map((vr, ri) => {
    const row = grid.points.filter((p) => p.ri === ri && p.valid).map((p) => p.d);
    return row.length ? { vr, dMin: Math.min(...row), dMax: Math.max(...row) } : { vr, dMin: null, dMax: null };
  });
}

/** Fit every output over the valid grid points. */
export function fitGrid(grid, degree) {
  const valid = grid.points.filter((p) => p.valid);
  const dMin = Math.min(...grid.dValues);
  const dMax = Math.max(...grid.dValues);
  const rMin = Math.min(...grid.rValues);
  const rMax = Math.max(...grid.rValues);
  const norm = {
    dMid: (dMin + dMax) / 2,
    dHalf: Math.max((dMax - dMin) / 2, 1e-9),
    rMid: (rMin + rMax) / 2,
    rHalf: Math.max((rMax - rMin) / 2, 1e-9),
  };
  const terms = basisTerms(degree);
  const pts = valid.map((p) => [p.d, p.vr]);
  const coefficients = {};
  const fitStats = {};
  for (const name of OUTPUTS) {
    const f = fitPoly(terms, norm, pts, valid.map((p) => p[name]));
    const c = [...f.coefficients];
    // Margins are only useful if they never overstate the truth: shift the fit down by its worst
    // overshoot so fitted margin <= simulated margin at every grid point.
    let shift = 0;
    if (MARGINS.has(name) && f.maxOver > 0) {
      shift = f.maxOver;
      c[0] -= shift;
    }
    coefficients[name] = c;
    fitStats[name] = { rms: f.rms, maxAbs: f.maxAbs, conservativeShift: shift };
  }
  return { norm, terms, coefficients, fitStats };
}

function grid2d(grid, name) {
  const at = new Map(grid.points.map((p) => [`${p.ri},${p.di}`, p]));
  return grid.rValues.map((_, ri) =>
    grid.dValues.map((__, di) => {
      const p = at.get(`${ri},${di}`);
      return p && p.valid ? round(p[name]) : null;
    }),
  );
}

/** Assemble the exported map. */
export function buildShotMap(config, mech, grid, configText = JSON.stringify(config)) {
  const degree = config.fit?.degree ?? 2;
  const fit = fitGrid(grid, degree);
  const rows = rowRanges(grid);
  const valid = grid.points.filter((p) => p.valid);
  const vtGlobal = valid.length ? Math.min(...valid.map((p) => p.vtMax)) : 0;
  const raw = {};
  for (const name of [...OUTPUTS, 'vtMax', 'apex', 'score']) raw[name] = grid2d(grid, name);
  const map = {
    schema: SCHEMA,
    name: config.name,
    kind: config.kind,
    target: config.kind === 'hub' ? 'HUB' : config.target.map((v) => round(v)),
    generator: { tool: 'tools/shotsim', configHash: hash(configText) },
    mechanism: mech,
    physics: config.physics,
    geometry: config.kind === 'hub' ? config.hub : config.feed,
    selection: config.selection,
    sotm: config.sotm,
    angleRangeDeg: [round(grid.angleRange.min), round(grid.angleRange.max)],
    speedRangeMps: [round(grid.speedRange.min), round(grid.speedRange.max)],
    grid: { d: grid.dValues, vr: grid.rValues },
    normalization: {
      dMid: round(fit.norm.dMid),
      dHalf: round(fit.norm.dHalf),
      rMid: round(fit.norm.rMid),
      rHalf: round(fit.norm.rHalf),
    },
    degree,
    basis: fit.terms,
    outputs: OUTPUTS,
    coefficients: Object.fromEntries(OUTPUTS.map((n) => [n, fit.coefficients[n].map((c) => round(c, 9))])),
    fitStats: Object.fromEntries(
      OUTPUTS.map((n) => [
        n,
        {
          rms: round(fit.fitStats[n].rms),
          maxAbs: round(fit.fitStats[n].maxAbs),
          conservativeShift: round(fit.fitStats[n].conservativeShift),
        },
      ]),
    ),
    bounds: {
      dMin: Math.min(...grid.dValues),
      dMax: Math.max(...grid.dValues),
      rMin: Math.min(...grid.rValues),
      rMax: Math.max(...grid.rValues),
    },
    rows: rows.map((r) => ({ vr: r.vr, dMin: r.dMin, dMax: r.dMax })),
    tangentialLimit: {
      global: round(vtGlobal),
      absMax: config.tangential.absMax,
      maxBearingRateRadPerSec: config.tangential.maxBearingRateRadPerSec ?? null,
    },
    // A global polynomial cannot follow the corner cuts or the hood-limit kink everywhere; when it
    // misses the simulated shots by more than this, the robot should interpolate the raw grid.
    recommendedMode:
      fit.fitStats.angleDeg.maxAbs <= (config.fit?.maxAngleErrorDeg ?? 0.5) &&
      fit.fitStats.speed.maxAbs <= (config.fit?.maxSpeedErrorMps ?? 0.05)
        ? 'POLY'
        : 'GRID',
    validCount: valid.length,
    totalCount: grid.points.length,
    rawGrid: raw,
  };
  if (config.kind !== 'hub' && config.feed.crossing) {
    // The robot picks between a "feed" and a "net feed" map by checking whether the line from the
    // turret to the target crosses these footprints (blue-alliance field coordinates, grown by the
    // ball radius exactly as the generator classified its launch bearings).
    map.crossing = {
      mode: config.feed.crossing.mode,
      boxes: crossingBoxes(config.feed, config.physics.ballDiameter / 2).map((o) => ({
        name: o.name,
        x: o.x.map((v) => round(v)),
        y: o.y.map((v) => round(v)),
      })),
    };
  }
  return map;
}
