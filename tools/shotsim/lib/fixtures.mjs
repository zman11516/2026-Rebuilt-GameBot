// Parity fixtures for the Java tests: a small hub map plus the values this JS reference produces
// for map evaluation and the shoot-on-the-move solver. The Java side must match them.

import { buildGrid } from './grid.mjs';
import { buildShotMap } from './export.mjs';
import { evalMap } from './mapeval.mjs';
import { solve } from './sotm.mjs';
import { hoodRotForAngle, angleForHoodRot, rpsForExitSpeed, exitSpeedForRps } from './mechanism.mjs';

export function makeFixtures(hubConfig, mech, configText) {
  const small = {
    ...hubConfig,
    name: 'fixture_hub',
    grid: {
      distance: { min: 1.5, max: 5.5, step: 0.5 },
      radialVelocity: { min: -1.0, max: 1.0, step: 0.5 },
    },
  };
  const grid = buildGrid(small, mech);
  const map = buildShotMap(small, mech, grid, configText);

  const evals = [];
  const probes = [
    [2.0, 0.0],
    [2.37, 0.31],
    [3.9, -0.62],
    [5.1, 0.85],
    [1.2, 0.0],
    [6.0, 0.0],
    [3.0, 1.4],
    [1.55, 0.95],
  ];
  for (const mode of ['POLY', 'GRID']) {
    for (const [d, vr] of probes) evals.push({ d, vr, mode, expected: evalMap(map, d, vr, mode) });
  }

  const target = [4.62, 4.04];
  const states = [
    { name: 'static', x: 2.0, y: 4.04, heading: 0.0, vx: 0, vy: 0, omega: 0 },
    { name: 'strafe_left', x: 2.0, y: 4.04, heading: 0.0, vx: 0, vy: 1.0, omega: 0 },
    { name: 'driving_in_rotated', x: 1.5, y: 2.5, heading: 1.1, vx: 0.8, vy: -0.3, omega: 0.4 },
    { name: 'backing_away_spinning', x: 6.8, y: 5.6, heading: -2.4, vx: -0.6, vy: 0.5, omega: -1.2 },
    { name: 'too_close', x: 4.0, y: 4.04, heading: 0.3, vx: 0, vy: 0, omega: 0 },
  ];
  const solves = [];
  for (const mode of ['POLY', 'GRID']) {
    for (const s of states) {
      const r = solve(s, target, map, mech, { latency: 0.05, turretOffset: mech.turretOffset, mode });
      solves.push({
        name: s.name,
        mode,
        state: s,
        target,
        latency: 0.05,
        expected: {
          valid: r.valid,
          reasons: r.reasons,
          d: r.d,
          vr: r.vr,
          vt: r.vt,
          turretYawDeg: r.turretYawDeg,
          elevationDeg: r.elevationDeg,
          exitSpeed: r.exitSpeed,
          hoodRot: r.hoodRot,
          flywheelRps: r.flywheelRps,
          tof: r.tof,
        },
      });
    }
  }

  const conversions = [0, 0.1, 0.33, 0.6].map((rot) => ({ hoodRot: rot, angleDeg: angleForHoodRot(mech, rot) }));
  conversions.push({ angleDeg: 80, hoodRot: hoodRotForAngle(mech, 80) }, { angleDeg: 40, hoodRot: hoodRotForAngle(mech, 40) });
  const speeds = [5, 8.5, 12].map((v) => ({ exitSpeed: v, rotorRps: rpsForExitSpeed(mech, v) }));
  speeds.push({ rotorRps: 60, exitSpeed: exitSpeedForRps(mech, 60) });

  return {
    'fixture_hub.json': map,
    'fixture_expected.json': { evals, solves, conversions, speeds, mechanism: mech },
  };
}
