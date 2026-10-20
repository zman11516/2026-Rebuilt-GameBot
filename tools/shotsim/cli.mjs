#!/usr/bin/env node
// Shot-map generator CLI.
//
//   node tools/shotsim/cli.mjs generate             all maps -> src/main/deploy/shotmaps/*.json
//                                                   + src/main/deploy/shooter_calibration.json
//   node tools/shotsim/cli.mjs generate --only hub  one config (hub | feed | netfeed)
//   node tools/shotsim/cli.mjs calibrate [--tof] [--write] [--data tools/shotsim/data/tuned_hub.csv]
//   node tools/shotsim/cli.mjs point --config hub --d 3 --vr 0
//   node tools/shotsim/cli.mjs fixtures             Java parity fixtures -> src/test/resources/shotmap/

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { MAP_CONFIGS, expandConfig, robotCalibration, stripDocs } from './lib/config.mjs';
import { buildGrid, makeContext, solvePoint } from './lib/grid.mjs';
import { feedProfile } from './lib/targets.mjs';
import { buildShotMap } from './lib/export.mjs';
import { calibrate, parseTunedCsv } from './lib/calibrate.mjs';
import { makeFixtures } from './lib/fixtures.mjs';

const TOOL = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(TOOL, '..', '..');
const DEPLOY = path.join(ROOT, 'src', 'main', 'deploy');

function args(argv) {
  const out = { _: [] };
  for (let i = 0; i < argv.length; i++) {
    if (argv[i].startsWith('--')) {
      const k = argv[i].slice(2);
      const v = argv[i + 1] && !argv[i + 1].startsWith('--') ? argv[++i] : true;
      out[k] = v;
    } else out._.push(argv[i]);
  }
  return out;
}

const readJson = (p) => JSON.parse(fs.readFileSync(p, 'utf8'));
const writeJson = (p, v) => {
  fs.mkdirSync(path.dirname(p), { recursive: true });
  fs.writeFileSync(p, JSON.stringify(v, null, 1) + '\n');
};

function loadConfig(nameOrPath) {
  const file = fs.existsSync(nameOrPath) ? nameOrPath : path.join(TOOL, 'configs', nameOrPath.endsWith('.json') ? nameOrPath : `${nameOrPath}.json`);
  const text = fs.readFileSync(file, 'utf8');
  const config = JSON.parse(text);
  const mech = stripDocs(readJson(path.join(path.dirname(file), config.mechanism)));
  return { config, mech, text, file };
}

function progressBar(label) {
  let last = -1;
  return (done, total) => {
    const pct = Math.floor((100 * done) / total);
    if (pct !== last && (pct % 10 === 0 || done === total)) {
      process.stderr.write(`  ${label}: ${pct}%\n`);
      last = pct;
    }
  };
}

function generate(opt) {
  const names = opt.only ? [`${opt.only}.json`] : MAP_CONFIGS;
  const outDir = opt.out ? path.resolve(opt.out) : path.join(DEPLOY, 'shotmaps');
  let hubLoaded = null;
  for (const name of names) {
    const { config, mech, text } = loadConfig(name);
    if (config.kind === 'hub') hubLoaded = { config, mech };
    for (const c of expandConfig(config)) {
      const t0 = Date.now();
      const grid = buildGrid(c, mech, { onProgress: progressBar(c.name) });
      const map = buildShotMap(c, mech, grid, text + JSON.stringify(mech));
      const file = path.join(outDir, `${c.name}.json`);
      writeJson(file, map);
      const s = map.fitStats;
      console.log(
        `${c.name}: ${map.validCount}/${map.totalCount} valid, angle rms ${s.angleDeg.rms} deg (max ${s.angleDeg.maxAbs}), ` +
          `speed rms ${s.speed.rms} m/s (max ${s.speed.maxAbs}), tof rms ${s.tof.rms} s, vt global ${map.tangentialLimit.global} m/s ` +
          `[${((Date.now() - t0) / 1000).toFixed(1)} s] -> ${path.relative(ROOT, file)}`,
      );
    }
  }
  if (!opt.only || opt.only === 'hub') {
    const hub = hubLoaded ?? loadConfig('hub.json');
    const file = path.join(DEPLOY, 'shooter_calibration.json');
    writeJson(file, robotCalibration(hub.mech, hub.config));
    console.log(`calibration -> ${path.relative(ROOT, file)}`);
  }
}

function runCalibrate(opt) {
  const { config, mech } = loadConfig('hub.json');
  const dataFile = opt.data ? path.resolve(opt.data) : path.join(TOOL, 'data', 'tuned_hub.csv');
  const rows = parseTunedCsv(fs.readFileSync(dataFile, 'utf8'));
  const res = calibrate(stripDocs(config), mech, rows, { useTof: !!opt.tof });
  console.log(`theta0 (launch angle at hood 0) = ${res.theta0Deg.toFixed(2)} deg`);
  console.log(`exit speed per rotor rps       = ${res.exitSpeedPerRotorRps.toFixed(4)} m/s/rps`);
  console.log(`cost ${res.cost.toFixed(2)} (${opt.tof ? 'distance + tof' : 'distance only'})`);
  console.log('  d      rps  hood   angle  speed   cross   miss   tof(model/table)  scores');
  for (const r of res.rows) {
    console.log(
      `  ${r.d.toFixed(3)}  ${String(r.rps).padStart(3)}  ${r.hoodRot.toFixed(2)}  ${r.angleDeg.toFixed(1).padStart(5)}  ${r.speed.toFixed(2)}  ` +
        `${(r.xCross ?? NaN).toFixed(2)}  ${(r.miss >= 0 ? '+' : '') + r.miss.toFixed(2)}  ${r.tofModel.toFixed(2)}/${r.tof ?? '-'}  ${r.scores ? 'yes' : 'NO ' + r.reason}`,
    );
  }
  console.log('  profile (best cost per theta0): ' + res.profile.map((p) => `${p.theta0Deg}:${p.cost.toFixed(1)}`).join('  '));
  if (opt.write) {
    const file = path.join(TOOL, 'configs', 'mechanism.json');
    const m = readJson(file);
    m.hood.theta0Deg = Math.round(res.theta0Deg * 100) / 100;
    m.flywheel.exitSpeedPerRotorRps = Math.round(res.exitSpeedPerRotorRps * 10000) / 10000;
    fs.writeFileSync(file, JSON.stringify(m, null, 2) + '\n');
    console.log(`updated ${path.relative(ROOT, file)}; re-run generate`);
  }
}

function point(opt) {
  const { config, mech } = loadConfig(opt.config ?? 'hub');
  const c = expandConfig(config).find((x) => !opt.target || x.name.endsWith(opt.target)) ?? expandConfig(config)[0];
  const ctx = makeContext(c, mech);
  const d = Number(opt.d ?? 3);
  const vr = Number(opt.vr ?? 0);
  const profile = c.kind === 'hub' ? null : feedProfile(c.feed, c.target, d, ctx.aero.r);
  const p = solvePoint(ctx, c, mech, d, vr, profile);
  console.log(JSON.stringify({ ...p, bands: p.bands.filter((b) => b.valid) }, null, 1));
}

function fixtures(opt) {
  const outDir = opt.out ? path.resolve(opt.out) : path.join(ROOT, 'src', 'test', 'resources', 'shotmap');
  const { config, mech, text } = loadConfig('hub.json');
  const f = makeFixtures(stripDocs(config), mech, text);
  for (const [name, v] of Object.entries(f)) writeJson(path.join(outDir, name), v);
  console.log(`fixtures -> ${path.relative(ROOT, outDir)}: ${Object.keys(f).join(', ')}`);
}

const a = args(process.argv.slice(2));
const cmd = a._[0] ?? 'generate';
if (cmd === 'generate') generate(a);
else if (cmd === 'calibrate') runCalibrate(a);
else if (cmd === 'point') point(a);
else if (cmd === 'fixtures') fixtures(a);
else {
  console.error(`unknown command ${cmd}`);
  process.exit(2);
}
