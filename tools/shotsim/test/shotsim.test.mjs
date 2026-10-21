// node --test tools/shotsim/test
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import { makeAero, integrate, trajectory } from '../lib/physics.mjs';
import { simulateHub, simulateFeed, feedProfile, crossesBox, obstacleBoxes } from '../lib/targets.mjs';
import { sweepAngles, angleList } from '../lib/sweep.mjs';
import { selectMaximin, selectMinPower, selectShot } from '../lib/select.mjs';
import { basisTerms, fitPoly, evalPoly } from '../lib/fit.mjs';
import { compensate, solve, poseExp } from '../lib/sotm.mjs';
import { evalMap } from '../lib/mapeval.mjs';
import { buildGrid, makeContext, solvePoint } from '../lib/grid.mjs';
import { buildShotMap, SCHEMA, OUTPUTS } from '../lib/export.mjs';
import { calibrate, parseTunedCsv, predictRows } from '../lib/calibrate.mjs';
import { stripDocs, expandConfig } from '../lib/config.mjs';
import { angleRange, releasePoint, hoodRotForAngle, angleForHoodRot } from '../lib/mechanism.mjs';
import { makeFixtures } from '../lib/fixtures.mjs';

const TOOL = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const ROOT = path.resolve(TOOL, '..', '..');
const read = (p) => JSON.parse(fs.readFileSync(path.join(TOOL, p), 'utf8'));
const hubText = fs.readFileSync(path.join(TOOL, 'configs/hub.json'), 'utf8');
const hub = stripDocs(JSON.parse(hubText));
const mech = stripDocs(read('configs/mechanism.json'));
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} expected ${b}, got ${a} (tol ${tol})`);

test('no drag/lift matches the analytic parabola', () => {
  const aero = makeAero({ ...hub.physics, dragCoefficient: 0, buoyancy: false, lift: { kL: 0, clMax: 0, spinRatio: 0 } });
  let at1 = null;
  integrate(aero, 0, 0.5, 4, 6, 0, (s) => {
    if (!at1 && s.t >= 1 - 1e-9) at1 = { ...s };
    return s.t >= 1.2;
  });
  near(at1.x, 4 * at1.t, 1e-9, 'x');
  near(at1.z, 0.5 + 6 * at1.t - 0.5 * hub.physics.gravity * at1.t ** 2, 1e-9, 'z');
});

test('drag shortens the range and backspin lift lengthens it', () => {
  const range = (phys) => {
    const pts = trajectory(makeAero(phys), 0, 0.5, 6, 6, 50);
    return pts[pts.length - 1].x;
  };
  const vac = range({ ...hub.physics, dragCoefficient: 0, buoyancy: false, lift: { kL: 0, clMax: 0, spinRatio: 0 } });
  const drag = range({ ...hub.physics, lift: { kL: 0, clMax: 0, spinRatio: 0 } });
  const both = range(hub.physics);
  assert.ok(drag < vac, 'drag < vacuum');
  assert.ok(both > drag, 'lift > drag only');
});

test('release point follows the hood (ball leaves tangent to the wheel)', () => {
  const r = releasePoint(mech, 90);
  near(r.x, mech.launcher.axleForward - mech.launcher.exitRadius, 1e-12);
  near(r.z, mech.launcher.axleHeight, 1e-12);
  const range = angleRange(mech);
  near(range.max, mech.hood.theta0Deg, 1e-9);
  near(hoodRotForAngle(mech, angleForHoodRot(mech, 0.37)), 0.37, 1e-12);
});

test('sweep bands contain only scoring speeds', () => {
  const aero = makeAero(hub.physics);
  const shot = (a, s) => simulateHub(aero, mech, hub.hub, 3.2, 0.5, a, s);
  const bands = sweepAngles(shot, [55, 62, 68], { min: 3, max: 14 }, 0.002);
  for (const b of bands) {
    assert.ok(b.valid, `angle ${b.angle} should have a scoring window`);
    assert.ok(shot(b.angle, b.sLow + 0.003).ok, 'just above sLow scores');
    assert.ok(!shot(b.angle, b.sLow - 0.01).ok, 'below sLow misses');
    assert.ok(shot(b.angle, b.sHigh - 0.003).ok, 'just below sHigh scores');
    assert.ok(!shot(b.angle, b.sHigh + 0.01).ok, 'above sHigh misses');
  }
});

test('maximin picks the widest point of a synthetic band', () => {
  // Speed window centred at 8 m/s, half-width shrinking away from 60 deg.
  const bands = angleList(50, 70, 1).map((a) => {
    const half = 0.6 - 0.04 * Math.abs(a - 60);
    return { angle: a, sLow: 8 - half, sHigh: 8 + half, valid: half > 0 };
  });
  const best = selectMaximin(bands, { sigmaSpeed: 0.1, sigmaAngleDeg: 1, angleMin: 50, angleMax: 70 });
  assert.equal(best.angle, 60);
  near(best.speed, 8, 1e-12);
  near(best.speedMarginLow, 0.6, 1e-12);
  assert.ok(best.angleMarginLow >= 9.5 && best.angleMarginHigh >= 9.5, 'all angles contain 8 m/s');
});

test('minPower picks the slowest shot that keeps every margin above the floor', () => {
  // Window lower edge falls toward flatter angles (slower), upper edge stays 1 m/s above it.
  const bands = angleList(50, 70, 1).map((a) => ({ angle: a, sLow: 6 + 0.05 * (a - 50), sHigh: 7 + 0.05 * (a - 50), valid: true }));
  const opts = { sigmaSpeed: 0.1, sigmaAngleDeg: 1, angleMin: 52, angleMax: 70, minMarginSigma: 2 };
  const low = selectMinPower(bands, opts);
  assert.equal(low.angle, 52, 'flattest angle in range is the slowest');
  near(low.speed, bands[2].sLow + 0.2, 1e-12, 'speed sits k sigma above the lower edge');
  assert.ok(low.speedMarginLow >= 0.2 - 1e-12 && low.speedMarginHigh >= 0.2 - 1e-12);
  assert.ok(low.angleMarginLow >= 2 && low.angleMarginHigh >= 2);
  assert.ok(low.speed < selectMaximin(bands, opts).speed);
  // A floor nothing can meet falls back to maximin.
  assert.equal(selectMinPower(bands, { ...opts, minMarginSigma: 10 }), null);
  const fb = selectShot(bands, 'minPower', { ...opts, minMarginSigma: 10 });
  assert.equal(fb.selection, 'maximin-fallback');
  assert.equal(fb.speed, selectMaximin(bands, opts).speed);
});

test('low-power feed shots are slower than maximin, keep 2 sigma margins and still land', () => {
  const feed = expandConfig(read('configs/feed.json'))[0];
  assert.equal(feed.selection.mode, 'minPower');
  const ctx = makeContext(feed, mech);
  const k = feed.selection.minMarginSigma;
  for (const d of [4, 6, 9]) {
    const profile = feedProfile(feed.feed, feed.target, d, ctx.aero.r);
    const low = solvePoint(ctx, feed, mech, d, 0, profile);
    const robust = solvePoint(ctx, { ...feed, selection: { ...feed.selection, mode: 'maximin' } }, mech, d, 0, profile);
    assert.equal(low.selection, 'minPower', `${d} m`);
    assert.ok(low.speed < robust.speed, `${d} m: ${low.speed} < ${robust.speed}`);
    for (const m of ['speedMarginLow', 'speedMarginHigh']) assert.ok(low[m] >= k * feed.selection.sigmaSpeed - 1e-9, `${d} m ${m}`);
    for (const m of ['angleMarginLow', 'angleMarginHigh']) assert.ok(low[m] >= k * feed.selection.sigmaAngleDeg - 1e-9, `${d} m ${m}`);
    assert.ok(simulateFeed(ctx.aero, mech, feed.feed, d, 0, low.angleDeg, low.speed, profile.intervals).ok, `${d} m lands`);
  }
});

test('every shot in a small low-power feed grid lands in simulation', () => {
  const feed = expandConfig(read('configs/netfeed.json'))[1];
  const small = { ...feed, grid: { distance: { min: 5, max: 11, step: 2 }, radialVelocity: { min: -1, max: 1, step: 1 } } };
  const grid = buildGrid(small, mech);
  const map = buildShotMap(small, mech, grid);
  assert.ok(map.selectionCounts.minPower > 0);
  const ctx = makeContext(small, mech);
  for (const p of grid.points.filter((q) => q.valid)) {
    const profile = feedProfile(small.feed, small.target, p.d, ctx.aero.r);
    assert.ok(simulateFeed(ctx.aero, mech, small.feed, p.d, p.vr, p.angleDeg, p.speed, profile.intervals).ok, `${p.d},${p.vr}`);
  }
});

test('polynomial fit recovers known coefficients', () => {
  const terms = basisTerms(2);
  const norm = { dMid: 3, dHalf: 2, rMid: 0, rHalf: 2 };
  const truth = [1.5, -0.7, 0.3, 0.25, -0.1, 0.05];
  const pts = [];
  const y = [];
  for (let d = 1; d <= 5; d += 0.5) {
    for (let r = -2; r <= 2; r += 0.5) {
      pts.push([d, r]);
      y.push(evalPoly(terms, truth, norm, d, r));
    }
  }
  const f = fitPoly(terms, norm, pts, y);
  f.coefficients.forEach((c, i) => near(c, truth[i], 1e-9, `coef ${i}`));
  near(f.rms, 0, 1e-9);
});

test("4414's worked example: (3, 0, 6) with 2 m/s sideways", () => {
  const c = compensate(3, 6, 2);
  near(c.yawOffsetDeg, -33.69, 0.005);
  near(c.elevationDeg, 59.0, 0.005); // video: hood 63.4 -> 59.0 deg
  near(c.speed, 7.0, 1e-9);
});

test('poseExp matches a straight drive and a pure rotation', () => {
  const a = poseExp(1, 2, Math.PI / 2, 0.5, 0, 0);
  near(a.x, 1, 1e-12);
  near(a.y, 2.5, 1e-12);
  const b = poseExp(0, 0, 0, 0, 0, 0.3);
  near(b.heading, 0.3, 1e-12);
  near(b.x, 0, 1e-12);
});

test('moving shot: launch vector + turret velocity has no sideways component', () => {
  const tiny = { ...hub, grid: { distance: { min: 1.5, max: 5.5, step: 0.5 }, radialVelocity: { min: -1, max: 1, step: 0.5 } } };
  const map = buildShotMap(tiny, mech, buildGrid(tiny, mech));
  const target = [4.62, 4.04];
  for (const s of [
    { x: 1.5, y: 2.5, heading: 1.1, vx: 0.8, vy: -0.3, omega: 0.4 },
    { x: 2.2, y: 5.0, heading: -0.4, vx: -0.2, vy: 0.9, omega: -0.7 },
  ]) {
    const r = solve(s, target, map, mech, { latency: 0, turretOffset: mech.turretOffset });
    const ux = (target[0] - r.turretPos[0]) / r.d;
    const uy = (target[1] - r.turretPos[1]) / r.d;
    // turret field velocity
    const [ox, oy] = mech.turretOffset;
    const rvx = s.vx - s.omega * oy;
    const rvy = s.vy + s.omega * ox;
    const fvx = rvx * Math.cos(s.heading) - rvy * Math.sin(s.heading);
    const fvy = rvx * Math.sin(s.heading) + rvy * Math.cos(s.heading);
    const side = -(r.launchVector[0] + fvx) * uy + (r.launchVector[1] + fvy) * ux;
    near(side, 0, 1e-9, 'field-frame tangential component');
  }
});

test('hood above its top angle is flagged only once the miss passes the map margin', () => {
  const map = JSON.parse(fs.readFileSync(path.join(ROOT, 'src/test/resources/shotmap/fixture_hub.json'), 'utf8'));
  const state = { x: 2.0, y: 4.04, heading: 0, vx: 0, vy: 0, omega: 0 };
  const opts = { latency: 0, turretOffset: mech.turretOffset, mode: 'GRID' };
  const r = solve(state, [4.62, 4.04], map, mech, opts);
  assert.ok(r.valid);
  // A lower theta0 stands in for the robot's negative hood trim (SOTM/HoodOffsetDeg).
  const room = angleRange(mech).max - r.elevationDeg + r.map.angleMarginLow;
  const trimmed = (dt) => ({ ...mech, hood: { ...mech.hood, theta0Deg: mech.hood.theta0Deg + dt } });
  assert.ok(solve(state, [4.62, 4.04], map, trimmed(-room + 0.05), opts).valid);
  const low = solve(state, [4.62, 4.04], map, trimmed(-room - 0.05), opts);
  assert.deepEqual(low.reasons, ['HOOD_ABOVE_MAX']);
  assert.equal(low.hoodRot, mech.hood.rotMin);
});

test('non-finite pose or speeds never produce a valid shot', () => {
  const map = JSON.parse(fs.readFileSync(path.join(ROOT, 'src/test/resources/shotmap/fixture_hub.json'), 'utf8'));
  const opts = { latency: 0.05, turretOffset: mech.turretOffset, mode: 'GRID' };
  for (const state of [
    { x: 2.0, y: 4.04, heading: 0, vx: NaN, vy: 0, omega: 0 },
    { x: NaN, y: 4.04, heading: 0, vx: 0, vy: 0, omega: 0 },
  ]) {
    const r = solve(state, [4.62, 4.04], map, mech, opts);
    assert.ok(!r.valid && r.reasons.includes('NON_FINITE_INPUT'));
  }
});

test('feed map: one launch point that cannot pass does not wipe out its distance', () => {
  const [c] = expandConfig(read('configs/feed.json'));
  const ctx = makeContext(c, mech);
  const at = (d) => solvePoint(ctx, c, mech, d, 0, feedProfile(c.feed, c.target, d, ctx.aero.r));
  // 5.75 m: the -60.7 deg launch point's ball would clip the hub corner, so the feed map leaves it out.
  const p = at(5.75);
  assert.ok(p.valid && !p.droppedBearings, `5.75 m: ${p.reason}`);
  // 3 m: the +12.1 deg launch point is 0.14 m from the trench it would have to clear.
  const q = at(3);
  assert.ok(q.valid, `3 m: ${q.reason}`);
  assert.deepEqual(q.droppedBearings.map(Math.round), [12]);
});

test('feed profiles: avoid/require split passes over the hub net', () => {
  const feed = stripDocs(read('configs/feed.json'));
  const net = stripDocs(read('configs/netfeed.json'));
  const left = feed.targets.left;
  const r = 0.075;
  const p = feedProfile(feed.feed, left, 6.5, r);
  const q = feedProfile(net.feed, left, 6.5, r);
  assert.ok(p.bearings.length > 0 && q.bearings.length > 0);
  assert.ok(!p.intervals.some((i) => i.name === 'hubNet'), 'feed map never crosses the net');
  assert.ok(q.intervals.some((i) => i.name === 'hubNet'), 'net feed map always crosses it');
  const boxes = obstacleBoxes(feed.feed);
  assert.ok(crossesBox(8, 3, left[0], left[1], boxes.find((b) => b.name === 'hubNet')));
  assert.equal(expandConfig(read('configs/feed.json')).map((c) => c.name).join(','), 'feed_left,feed_right');
});

test('calibration on the bundled tuned table', () => {
  const rows = parseTunedCsv(fs.readFileSync(path.join(TOOL, 'data/tuned_hub.csv'), 'utf8'));
  assert.equal(rows.length, 8);
  const res = calibrate(hub, mech, rows);
  assert.ok(res.theta0Deg > 68 && res.theta0Deg < 76, `theta0 ${res.theta0Deg}`);
  assert.ok(res.exitSpeedPerRotorRps > 0.135 && res.exitSpeedPerRotorRps < 0.16, `kv ${res.exitSpeedPerRotorRps}`);
  // With the committed mechanism.json knobs, every hand-tuned shot must still go in.
  for (const p of predictRows(hub, mech, rows, mech.hood.theta0Deg, mech.flywheel.exitSpeedPerRotorRps)) {
    assert.ok(p.scores, `tuned shot at ${p.d} m (${p.rps} rps, hood ${p.hoodRot}) should score: ${p.reason} miss ${p.miss}`);
  }
});

test('exported map: schema, grid round-trip, every valid grid shot scores in simulation', () => {
  const tiny = { ...hub, grid: { distance: { min: 2, max: 4, step: 0.5 }, radialVelocity: { min: -1, max: 1, step: 1 } } };
  const grid = buildGrid(tiny, mech);
  const map = buildShotMap(tiny, mech, grid);
  assert.equal(map.schema, SCHEMA);
  assert.deepEqual(map.outputs, OUTPUTS);
  assert.ok(['POLY', 'GRID'].includes(map.recommendedMode));
  const aero = makeAero(tiny.physics);
  for (const p of grid.points.filter((q) => q.valid)) {
    const e = evalMap(map, p.d, p.vr, 'GRID');
    near(e.angleDeg, p.angleDeg, 1e-6);
    near(e.speed, p.speed, 1e-6);
    assert.ok(simulateHub(aero, mech, tiny.hub, p.d, p.vr, p.angleDeg, p.speed).ok, `grid shot at ${p.d},${p.vr} scores`);
  }
});

test('Java parity fixtures are up to date with this code', () => {
  const dir = path.join(ROOT, 'src/test/resources/shotmap');
  const fresh = makeFixtures(hub, mech, hubText);
  for (const [name, v] of Object.entries(fresh)) {
    const onDisk = JSON.parse(fs.readFileSync(path.join(dir, name), 'utf8'));
    assert.deepEqual(onDisk, JSON.parse(JSON.stringify(v)), `${name} is stale: run node tools/shotsim/cli.mjs fixtures`);
  }
});
