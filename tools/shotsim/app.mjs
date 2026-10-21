// Shot simulator web UI (TideShot-style). Every number comes from ./lib, the same code the CLI uses
// for the committed maps; this file only wires the config panel, Web Workers and plots together.

import { MAP_CONFIGS, expandConfig, robotCalibration, stripDocs } from './lib/config.mjs';
import { axis, makeContext, solvePoint } from './lib/grid.mjs';
import { integrate } from './lib/physics.mjs';
import { launchState, feedProfile, obstacleBoxes, crossesBox, crossingBoxes } from './lib/targets.mjs';
import { probabilityOfScore } from './lib/select.mjs';
import { evalPoly } from './lib/fit.mjs';
import { OUTPUTS, fitGrid, buildShotMap } from './lib/export.mjs';
import { evalMap } from './lib/mapeval.mjs';
import { compensate } from './lib/sotm.mjs';
import { calibrate, parseTunedCsv } from './lib/calibrate.mjs';
import { angleRange, speedRange, hoodRotForAngle, rpsForExitSpeed } from './lib/mechanism.mjs';
import { COLORS, darkAxis, hasPlotly, layout, line, plot } from './lib/ui-plot.mjs';
import { buildForm } from './lib/ui-form.mjs';

const DEG = Math.PI / 180;
const PRACTICE_APEX = 3.5; // 12 ft practice ceiling less the ball and a margin (configs/hub.json _source)
// The hub net, drawn for context only (the hub model stops at the entry plane). Offset from the hub
// centre and height span match the hubNet obstacle in configs/feed.json.
const NET = { offset: 0.8575, zMin: 1.2637, zMax: 3.0571 };
const FIELD = { w: 16.541, h: 8.069 };
const UNITS = {
  angleDeg: 'deg',
  speed: 'm/s',
  tof: 's',
  angleMarginLow: 'deg',
  angleMarginHigh: 'deg',
  speedMarginLow: 'm/s',
  speedMarginHigh: 'm/s',
};

const $ = (id) => document.getElementById(id);
const clone = (v) => structuredClone(v);
const fx = (v, digits = 2) => (Number.isFinite(v) ? v.toFixed(digits) : '–');
const num = (id) => Number($(id).value);
const tr = (cells, cls = '') => `<tr${cls ? ` class="${cls}"` : ''}>${cells.map((c) => `<td>${c}</td>`).join('')}</tr>`;
const head = (cells) => `<tr>${cells.map((c) => `<th>${c}</th>`).join('')}</tr>`;

const state = {
  files: {}, // config files as loaded (hub.json, feed.json, netfeed.json)
  mechFile: null,
  mech: null, // editable mechanism, shared by every target
  configs: {}, // editable per-map configs (hub, feed_left, ...), from expandConfig
  fileApex: {},
  name: 'hub',
  tab: 'single',
  sweep: null, // { config, mech, grid, seconds, map } from the last full sweep
  job: null,
  maps: {}, // committed maps from /deploy/shotmaps/
  calib: null,
  robot: [9.0, 5.5],
};
const current = () => state.configs[state.name];
let refreshForm = () => {};

function banner(msg, kind = 'error') {
  const el = document.createElement('div');
  el.className = kind;
  el.textContent = msg;
  $('banner').append(el);
}

async function fetchJson(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return r.json();
}

function loadMap(name) {
  state.maps[name] ??= fetchJson(`deploy/shotmaps/${name}.json`).catch((err) => {
    delete state.maps[name];
    throw err;
  });
  return state.maps[name];
}

function resetConfigs() {
  state.mech = clone(state.mechFile);
  state.configs = {};
  for (const file of MAP_CONFIGS) {
    // Deep copy: expandConfig shares nested objects between the left and right feed targets.
    for (const c of expandConfig(state.files[file])) {
      state.configs[c.name] = clone(c);
      state.fileApex[c.name] = (c.kind === 'hub' ? c.hub : c.feed).apexMax;
    }
  }
  applyPracticeApex();
}

function applyPracticeApex() {
  const practice = $('practiceApex').checked;
  for (const c of Object.values(state.configs)) {
    (c.kind === 'hub' ? c.hub : c.feed).apexMax = practice ? PRACTICE_APEX : state.fileApex[c.name];
  }
}

/** Something in the config panel changed: refresh whatever depends on it. */
function configEdited() {
  updateStale();
  if (state.tab === 'single') runSingle();
  else if (state.tab === 'feed') renderFeed();
  else if (state.tab === 'export') renderExport();
}

function sweepStale() {
  const sw = state.sweep;
  if (!sw) return false;
  return (
    JSON.stringify(sw.config) !== JSON.stringify(state.configs[sw.config.name]) ||
    JSON.stringify(sw.mech) !== JSON.stringify(state.mech)
  );
}

function updateStale() {
  const sw = state.sweep;
  const text = !sw
    ? 'no full sweep yet'
    : `last sweep: ${sw.config.name}${sweepStale() ? ' (config changed since)' : ''}`;
  for (const el of document.querySelectorAll('.sweep-status')) {
    el.textContent = text;
    el.classList.toggle('stale', sweepStale());
  }
}

// ---------------------------------------------------------------- 1 Single Sweep

/** Flight from release until the ball comes down through zStop (or lands, or passes xMax). */
function flight(aero, mech, angle, s, vr, zStop, xMax) {
  const L = launchState(aero, mech, angle, s, vr);
  const x = [L.x0];
  const z = [L.z0];
  integrate(aero, L.x0, L.z0, L.vx0, L.vz0, L.spin, (st) => {
    x.push(st.x);
    z.push(st.z);
    return (st.pz >= zStop && st.z < zStop && st.vz < 0) || st.z < 0 || st.x > xMax;
  });
  return { x, z };
}

const seg = (x0, y0, x1, y1, color, width = 1, dash = 'solid') => ({
  type: 'line',
  x0,
  y0,
  x1,
  y1,
  line: { color, width, dash },
});

function hubShapes(hub, d, r) {
  const he = hub.entryHeight;
  const w = hub.halfWidth - r - hub.windowInset;
  const xf = d - hub.faceOffset;
  const xb = d + hub.faceOffset;
  const need = r + hub.lipClearance;
  return [
    { ...seg(0, he, 1, he, COLORS.low, 1, 'dash'), xref: 'paper' }, // entry plane
    {
      type: 'path',
      path: `M${xf},0 L${xf},${he} L${d - hub.halfWidth},${he} M${d + hub.halfWidth},${he} L${xb},${he} L${xb},0`,
      line: { color: COLORS.muted, width: 2 },
    },
    seg(d - hub.halfWidth, he, d + hub.halfWidth, he, COLORS.muted, 6), // opening
    seg(d - w, he, d + w, he, COLORS.high, 3), // where the ball centre may cross
    seg(xf, 0, xf, he, COLORS.low, 3), // front face
    seg(d + NET.offset, NET.zMin, d + NET.offset, NET.zMax, COLORS.muted, 2, 'dot'),
    {
      type: 'circle',
      x0: d - hub.halfWidth - need,
      x1: d - hub.halfWidth + need,
      y0: he - need,
      y1: he + need,
      line: { color: '#ffffff', width: 1.5 },
    },
  ];
}

function feedShapes(feed, d, profile, r) {
  const need = r + feed.obstacleClearance;
  const shapes = [
    seg(d - feed.zoneRadius, feed.landingHeight, d + feed.zoneRadius, feed.landingHeight, COLORS.high, 5),
  ];
  const annotations = [];
  const named = new Set();
  for (const o of profile.intervals) {
    shapes.push({
      type: 'rect',
      x0: o.x1,
      x1: o.x2,
      y0: 0,
      y1: o.zReq - need,
      fillcolor: 'rgba(127,138,163,0.12)',
      line: { width: 0 },
    });
    shapes.push(seg(o.x1, o.zReq, o.x2, o.zReq, COLORS.low, 1, 'dot'));
    if (!named.has(o.name)) {
      named.add(o.name);
      annotations.push({
        x: (o.x1 + o.x2) / 2,
        y: o.zReq,
        text: o.name,
        showarrow: false,
        yshift: 10,
        font: { size: 10, color: COLORS.muted },
      });
    }
  }
  return { shapes, annotations };
}

function runSingle() {
  const config = current();
  const mech = state.mech;
  const d = num('s-d');
  const vr = num('s-vr');
  if (!Number.isFinite(d) || !Number.isFinite(vr) || d <= 0) return;
  const t0 = performance.now();
  const ctx = makeContext(config, mech);
  const shotFor = ctx.shotFor;
  let shots = 0;
  ctx.shotFor = (...a) => {
    const shot = shotFor(...a);
    return (angle, s) => (shots++, shot(angle, s));
  };
  const r = ctx.aero.r;
  const profile = config.kind === 'hub' ? null : feedProfile(config.feed, config.target, d, r);
  const p = solvePoint(ctx, config, mech, d, vr, profile);
  const ms = performance.now() - t0;

  const inRange = p.bands.filter((b) => b.valid && b.angle >= ctx.range.min - 1e-9 && b.angle <= ctx.range.max + 1e-9);
  $('s-angles').textContent = inRange.length;
  $('s-shots').textContent = shots.toLocaleString();
  $('s-time').textContent = `${ms.toFixed(0)} ms`;

  // (a) trajectory fan
  const zStop = config.kind === 'hub' ? config.hub.entryHeight : config.feed.landingHeight;
  const xMax = d + (config.kind === 'hub' ? config.hub.faceOffset : config.feed.zoneRadius) + 1;
  const fly = (angle, s) => flight(ctx.aero, mech, angle, s, vr, zStop, xMax);
  const fan = (key, color) => {
    const x = [];
    const y = [];
    for (const b of inRange) {
      const f = fly(b.angle, b[key]);
      x.push(...f.x, null);
      y.push(...f.z, null);
    }
    return line(x, y, color, 1, { opacity: 0.6 });
  };
  const traces = [fan('sLow', COLORS.low), fan('sHigh', COLORS.high)];
  if (p.valid) {
    const f = fly(p.angleDeg, p.speed);
    traces.push(line(f.x, f.z, COLORS.chosen, 4, { hoverinfo: 'x+y' }));
  }
  const extra =
    config.kind === 'hub'
      ? { shapes: hubShapes(config.hub, d, r), annotations: [] }
      : feedShapes(config.feed, d, profile, r);
  plot(
    $('s-traj'),
    traces,
    layout({
      title: { text: `${config.name}: d = ${d} m, vr = ${vr} m/s`, font: { size: 13 } },
      xaxis: { title: 'Distance from turret axis (m)', range: [-0.2, xMax] },
      yaxis: { title: 'Height (m)', scaleanchor: 'x', scaleratio: 1, rangemode: 'tozero', constrain: 'domain' },
      ...extra,
    }),
  );

  // (b) valid shot region
  const xs = p.bands.map((b) => b.angle);
  const lo = p.bands.map((b) => (b.valid ? b.sLow : null));
  const hi = p.bands.map((b) => (b.valid ? b.sHigh : null));
  const region = [
    {
      type: 'scatter',
      mode: 'lines+markers',
      x: xs,
      y: lo,
      name: 'lower limit',
      line: { color: COLORS.low },
      marker: { size: 4 },
    },
    {
      type: 'scatter',
      mode: 'lines+markers',
      x: xs,
      y: hi,
      name: 'upper limit',
      fill: 'tonexty',
      fillcolor: 'rgba(87,171,90,0.18)',
      line: { color: COLORS.high },
      marker: { size: 4 },
    },
  ];
  if (p.valid) {
    const a = p.angleDeg;
    const s = p.speed;
    region.push(
      line([a, a], [s - p.speedMarginLow, s + p.speedMarginHigh], COLORS.speed, 3),
      line([a - p.angleMarginLow, a + p.angleMarginHigh], [s, s], COLORS.angle, 3),
      {
        type: 'scatter',
        mode: 'markers',
        x: [a],
        y: [s],
        marker: { symbol: 'star', size: 16, color: '#ffffff', line: { color: COLORS.chosen, width: 1.5 } },
      },
    );
  }
  const hood = [ctx.range.min, ctx.range.max].map((x) => ({
    ...seg(x, 0, x, 1, COLORS.muted, 1, 'dash'),
    yref: 'paper',
  }));
  plot(
    $('s-region'),
    region,
    layout({
      title: { text: 'Valid Shot Region (Angle vs Speed)', font: { size: 13 } },
      xaxis: { title: 'Launch angle (deg)' },
      yaxis: { title: 'Exit speed (m/s)' },
      shapes: hood,
    }),
  );

  // (c) readout
  const out = $('s-readout');
  if (!p.valid) {
    out.innerHTML = `<div class="bad">No scoring shot (${p.reason}).</div>`;
    return;
  }
  const pScore = probabilityOfScore(shotFor(d, vr, profile), p.angleDeg, p.speed, {
    sigmaSpeed: config.selection.sigmaSpeed,
    sigmaAngleDeg: config.selection.sigmaAngleDeg,
    samples: 256,
  });
  const bearings = profile
    ? `<div><b>Launch bearings:</b> ${profile.bearings.length}/${config.feed.bearingSamples} in the worst-case profile` +
      (p.droppedBearings ? ` (${p.droppedBearings.length} hugging an obstacle with no shot left out)` : '') +
      '</div>'
    : '';
  const rule =
    p.selection === 'minPower'
      ? `low power (slowest shot with every margin ≥ ${config.selection.minMarginSigma ?? 2}σ)`
      : p.selection === 'maximin-fallback'
        ? 'most robust (fallback: no low-power shot meets the margin floor here)'
        : 'most robust (maximin)';
  out.innerHTML = `
    <div><b class="c-chosen">Optimal:</b> ${fx(p.angleDeg, 1)}° @ ${fx(p.speed)} m/s</div>
    <div><b>Rule:</b> ${rule}</div>
    <div><b class="c-speed">Speed:</b> −${fx(p.speedMarginLow)}/+${fx(p.speedMarginHigh)} m/s</div>
    <div><b class="c-angle">Angle:</b> −${fx(p.angleMarginLow, 1)}°/+${fx(p.angleMarginHigh, 1)}°</div>
    <div><b>TOF:</b> ${fx(p.tof, 3)} s &nbsp; <b>Apex:</b> ${fx(p.apex)} m</div>
    <div><b>Hood:</b> ${fx(hoodRotForAngle(mech, p.angleDeg), 3)} rot &nbsp;
      <b>Flywheel:</b> ${fx(rpsForExitSpeed(mech, p.speed), 1)} rotor rps</div>
    <div><b>Score:</b> ${fx(p.score)} σ (weakest margin) &nbsp;
      <b>P(score):</b> ${fx(100 * pScore, 1)}% <i>(256 Monte Carlo shots)</i></div>
    <div><b>vt max:</b> ${fx(p.vtMax)} m/s</div>${bearings}`;
}

// ---------------------------------------------------------------- 2 Full Sweep

const HEATMAPS = [
  ['angleDeg', 'Launch angle (deg)', (p) => p.angleDeg],
  ['speed', 'Exit speed (m/s)', (p) => p.speed],
  ['rps', 'Flywheel (rotor rps)', (p, mech) => rpsForExitSpeed(mech, p.speed)],
  ['tof', 'Time of flight (s)', (p) => p.tof],
  ['score', 'Weakest margin (σ)', (p) => p.score],
  ['vtMax', 'Tangential limit (m/s)', (p) => p.vtMax],
];

function runSweep() {
  if (state.job) return;
  const config = clone(current());
  const mech = clone(state.mech);
  const dValues = axis(config.grid.distance);
  const rValues = axis(config.grid.radialVelocity);
  const total = dValues.length * rValues.length;
  const n = Math.max(1, Math.min(rValues.length, (navigator.hardwareConcurrency || 2) - 1));
  const done = new Array(n).fill(0);
  const points = [];
  const t0 = performance.now();
  let finished = 0;
  const workers = [];
  const stop = (msg) => {
    workers.forEach((w) => w.terminate());
    state.job = null;
    $('g-run').disabled = false;
    $('g-cancel').disabled = true;
    if (msg) $('g-info').textContent = msg;
  };
  const progress = () => {
    const k = done.reduce((a, b) => a + b, 0);
    $('g-bar').style.width = `${(100 * k) / total}%`;
    $('g-info').textContent =
      `${config.name}: ${k}/${total} points · ${n} worker${n > 1 ? 's' : ''} · ${((performance.now() - t0) / 1000).toFixed(1)} s`;
  };
  for (let k = 0; k < n; k++) {
    const w = new Worker(new URL('./worker.mjs', import.meta.url), { type: 'module' });
    w.onmessage = (e) => {
      const m = e.data;
      if (m.type === 'progress') {
        done[k] = m.done;
        progress();
      } else if (m.type === 'done') {
        points.push(...m.points);
        if (++finished < n) return;
        points.sort((a, b) => a.ri - b.ri || a.di - b.di);
        const grid = { dValues, rValues, points, angleRange: angleRange(mech), speedRange: speedRange(mech) };
        state.sweep = { config, mech, grid, seconds: (performance.now() - t0) / 1000, map: null };
        $('f-degree').value = config.fit?.degree ?? 2;
        progress();
        stop();
        updateStale();
        renderSweep();
      } else stop(`sweep failed: ${m.message}`);
    };
    w.onerror = (e) => stop(`worker failed: ${e.message || 'could not load worker.mjs'}`);
    w.postMessage({ config, mech, rows: rValues.map((_, ri) => ri).filter((ri) => ri % n === k) });
    workers.push(w);
  }
  state.job = { cancel: () => stop('cancelled') };
  $('g-run').disabled = true;
  $('g-cancel').disabled = false;
  $('g-bar').style.width = '0%';
  progress();
}

function renderSweep() {
  const sw = state.sweep;
  const box = $('g-maps');
  if (!sw) {
    box.innerHTML = '<p class="hint">Run a full sweep of the selected target (grid ranges from the config panel).</p>';
    $('g-summary').textContent = '';
    return;
  }
  const { grid, mech } = sw;
  const valid = grid.points.filter((p) => p.valid);
  $('g-summary').textContent =
    `${sw.config.name}: ${valid.length}/${grid.points.length} grid points have a scoring shot · ` +
    `${sw.seconds.toFixed(1)} s · global vt limit ${fx(Math.min(...valid.map((p) => p.vtMax)))} m/s · ` +
    'click a cell to open it in Single Sweep';
  box.innerHTML = '';
  for (const [key, title, value] of HEATMAPS) {
    const el = document.createElement('div');
    el.className = 'plot heat';
    el.id = `g-${key}`;
    box.append(el);
    const z = grid.rValues.map(() => grid.dValues.map(() => null));
    for (const p of grid.points) if (p.valid) z[p.ri][p.di] = value(p, mech);
    plot(
      el,
      [
        {
          type: 'heatmap',
          x: grid.dValues,
          y: grid.rValues,
          z,
          colorscale: 'Viridis',
          hoverongaps: false,
          hovertemplate: 'd %{x} m<br>vr %{y} m/s<br>%{z:.3f}<extra></extra>',
        },
      ],
      layout({
        title: { text: title, font: { size: 13 } },
        plot_bgcolor: COLORS.invalid,
        xaxis: { title: 'Distance (m)' },
        yaxis: { title: 'Radial velocity (m/s)' },
        margin: { l: 56, r: 8, t: 36, b: 44 },
      }),
    );
    if (hasPlotly()) {
      el.on('plotly_click', (ev) => {
        const pt = ev.points[0];
        if (state.name !== sw.config.name) {
          state.name = sw.config.name;
          $('target').value = state.name;
          refreshForm();
        }
        $('s-d').value = pt.x;
        $('s-vr').value = pt.y;
        showTab('single');
      });
    }
  }
}

// ---------------------------------------------------------------- 3 Fit

/** Largest fit errors that still allow POLY (same defaults as export.mjs buildShotMap). */
const polyLimits = (config) => ({
  angle: config.fit?.maxAngleErrorDeg ?? 0.5,
  speed: config.fit?.maxSpeedErrorMps ?? 0.05,
});

function recommendedMode(fit, config) {
  const lim = polyLimits(config);
  return fit.fitStats.angleDeg.maxAbs <= lim.angle && fit.fitStats.speed.maxAbs <= lim.speed ? 'POLY' : 'GRID';
}

function renderFit() {
  const sw = state.sweep;
  if (!sw) {
    $('f-mode').textContent = 'Run a full sweep first.';
    $('f-table').innerHTML = '';
    $('f-degrees').innerHTML = '';
    return;
  }
  const degree = num('f-degree');
  // The selected degree is what Export writes, for the sweep and for later sweeps of this target.
  sw.config.fit = { ...sw.config.fit, degree };
  state.configs[sw.config.name].fit = { ...state.configs[sw.config.name].fit, degree };
  sw.map = null;
  let fit;
  try {
    fit = fitGrid(sw.grid, degree);
  } catch (err) {
    $('f-mode').textContent = `Fit failed: ${err.message}`;
    return;
  }
  const mode = recommendedMode(fit, sw.config);
  const lim = polyLimits(sw.config);
  $('f-mode').innerHTML =
    `${sw.config.name}, degree ${degree} (${fit.terms.length} terms): recommended mode ` +
    `<b class="${mode === 'POLY' ? 'c-high' : 'c-speed'}">${mode}</b> ` +
    `<i>(POLY needs angle max |err| ≤ ${lim.angle}° and speed ≤ ${lim.speed} m/s)</i>`;
  $('f-table').innerHTML =
    head(['output', 'rms', 'max |err|', 'conservative shift']) +
    OUTPUTS.map((name) => {
      const s = fit.fitStats[name];
      return tr([
        name,
        `${fx(s.rms, 4)} ${UNITS[name]}`,
        `${fx(s.maxAbs, 4)} ${UNITS[name]}`,
        fx(s.conservativeShift, 4),
      ]);
    }).join('');
  const rows = [2, 3, 4, 5].map((k) => {
    try {
      const f = fitGrid(sw.grid, k);
      const s = f.fitStats;
      const cells = [k, `${fx(s.angleDeg.maxAbs, 3)}°`, `${fx(s.speed.maxAbs, 3)} m/s`, recommendedMode(f, sw.config)];
      return tr(cells, k === degree ? 'sel' : '');
    } catch (err) {
      return `<tr><td>${k}</td><td colspan="3">${err.message}</td></tr>`;
    }
  });
  $('f-degrees').innerHTML = head(['degree', 'angle max', 'speed max', 'mode']) + rows.join('');

  const { grid } = sw;
  const z = grid.rValues.map(() => grid.dValues.map(() => null));
  const valid = grid.points.filter((p) => p.valid);
  for (const p of valid) z[p.ri][p.di] = evalPoly(fit.terms, fit.coefficients.angleDeg, fit.norm, p.d, p.vr);
  const scene = (title) => ({ ...darkAxis(), title });
  plot(
    $('f-surface'),
    [
      {
        type: 'surface',
        x: grid.dValues,
        y: grid.rValues,
        z,
        colorscale: 'Viridis',
        opacity: 0.85,
        showscale: false,
        hoverinfo: 'skip',
      },
      {
        type: 'scatter3d',
        mode: 'markers',
        x: valid.map((p) => p.d),
        y: valid.map((p) => p.vr),
        z: valid.map((p) => p.angleDeg),
        marker: { size: 3.5, color: COLORS.high },
        hovertemplate: 'd %{x} m<br>vr %{y} m/s<br>simulated %{z:.2f}°<extra></extra>',
      },
    ],
    layout({
      title: { text: 'Fitted launch angle vs simulated grid points', font: { size: 13 } },
      margin: { l: 0, r: 0, t: 36, b: 0 },
      scene: {
        xaxis: scene('Distance (m)'),
        yaxis: scene('Radial (m/s)'),
        zaxis: scene('Angle (deg)'),
        bgcolor: COLORS.bg,
        aspectmode: 'cube',
        camera: { eye: { x: 1.6, y: -1.6, z: 0.9 } },
      },
    }),
  );
}

function sweepMap() {
  const sw = state.sweep;
  sw.map ??= buildShotMap(sw.config, sw.mech, sw.grid, JSON.stringify(sw.config) + JSON.stringify(sw.mech));
  return sw.map;
}

// ---------------------------------------------------------------- 4 Moving Shot

function svgEl(tag, attrs = {}, text) {
  const el = document.createElementNS('http://www.w3.org/2000/svg', tag);
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  if (text !== undefined) el.textContent = text;
  return el;
}

function arrow(svg, x1, y1, x2, y2, color, width = 3) {
  svg.append(svgEl('line', { x1, y1, x2, y2, stroke: color, 'stroke-width': width }));
  const a = Math.atan2(y2 - y1, x2 - x1);
  const h = 10;
  const pts = [
    [x2, y2],
    [x2 - h * Math.cos(a - 0.4), y2 - h * Math.sin(a - 0.4)],
    [x2 - h * Math.cos(a + 0.4), y2 - h * Math.sin(a + 0.4)],
  ];
  svg.append(svgEl('polygon', { points: pts.map((p) => p.join(',')).join(' '), fill: color }));
}

/** Top-down view of the hub frame: u (toward the target) points up the page, t (left of the line of sight) left. */
function drawVectors(svg, H, vr, vt) {
  svg.innerHTML = '';
  const W = 320;
  const pad = 56;
  const pts = [
    [0, 0],
    [H, -vt],
    [H + vr, 0],
  ];
  const us = pts.map((p) => p[0]);
  const ts = pts.map((p) => p[1]);
  const k = (W - 2 * pad) / Math.max(Math.max(...us) - Math.min(...us), Math.max(...ts) - Math.min(...ts), 1);
  const uc = (Math.max(...us) + Math.min(...us)) / 2;
  const tc = (Math.max(...ts) + Math.min(...ts)) / 2;
  const P = (u, t) => [W / 2 - (t - tc) * k, W / 2 + 12 - (u - uc) * k];
  const [ox, oy] = P(0, 0);
  const [sx, sy] = P(H, -vt);
  const [gx, gy] = P(H + vr, 0);
  svg.append(svgEl('line', { x1: ox, y1: oy, x2: ox, y2: 16, stroke: COLORS.muted, 'stroke-dasharray': '4 4' }));
  svg.append(svgEl('text', { x: ox + 6, y: 16, fill: COLORS.muted }, 'to target'));
  arrow(svg, ox, oy, sx, sy, COLORS.chosen);
  arrow(svg, sx, sy, gx, gy, COLORS.speed, 2);
  arrow(svg, ox, oy, gx, gy, COLORS.high);
  // Label each arrow at its midpoint, pushed away from the middle of the triangle.
  const mx = (ox + sx + gx) / 3;
  const my = (oy + sy + gy) / 3;
  const label = (x1, y1, x2, y2, color, text) => {
    const len = Math.hypot(x2 - x1, y2 - y1) || 1;
    let nx = -(y2 - y1) / len;
    let ny = (x2 - x1) / len;
    const cx = (x1 + x2) / 2;
    const cy = (y1 + y2) / 2;
    if (nx * (cx - mx) + ny * (cy - my) < 0) [nx, ny] = [-nx, -ny];
    const anchor = nx > 0.3 ? 'start' : nx < -0.3 ? 'end' : 'middle';
    svg.append(svgEl('text', { x: cx + 10 * nx, y: cy + 10 * ny + 4, fill: color, 'text-anchor': anchor }, text));
  };
  label(ox, oy, sx, sy, COLORS.chosen, 'shooter (H, −vt)');
  label(sx, sy, gx, gy, COLORS.speed, 'robot (vr, vt)');
  label(ox, oy, gx, gy, COLORS.high, 'field (H + vr, 0)');
  svg.append(svgEl('text', { x: ox, y: oy + 18, fill: COLORS.muted, 'text-anchor': 'middle' }, 'turret'));
}

async function runMove() {
  const source = $('m-source').value;
  const d = num('m-d');
  const vr = num('m-vr');
  const vt = num('m-vt');
  let map;
  try {
    if (source === 'sweep') {
      if (!state.sweep) throw new Error('run a full sweep first');
      map = sweepMap();
    } else map = await loadMap(source);
  } catch (err) {
    $('m-table').innerHTML = `<tr><td class="bad">Could not load map: ${err.message}</td></tr>`;
    return;
  }
  const mech = state.mech;
  const hood = angleRange(mech);
  const cols = ['POLY', 'GRID'].map((mode) => {
    const m = evalMap(map, d, vr, mode);
    const H = m.speed * Math.cos(m.angleDeg * DEG);
    const V = m.speed * Math.sin(m.angleDeg * DEG);
    const c = compensate(H, V, vt);
    const rps = rpsForExitSpeed(mech, c.speed);
    // Same checks as sotm.mjs solve().
    const reasons = [];
    if (!m.valid) reasons.push(m.reason);
    if (Math.abs(vt) > m.vtMax) reasons.push('TANGENTIAL_TOO_FAST');
    if (c.elevationDeg < hood.min - 1e-6) reasons.push('HOOD_BELOW_MIN');
    if (c.elevationDeg > hood.max + m.angleMarginLow + 1e-6) reasons.push('HOOD_ABOVE_MAX');
    if (rps > mech.flywheel.rotorRpsMax + 1e-6) reasons.push('FLYWHEEL_TOO_FAST');
    return { mode, m, H, V, c, rps, hoodRot: hoodRotForAngle(mech, c.elevationDeg), reasons };
  });
  const row = (label, fn, cls = '') =>
    `<tr class="${cls}"><th>${label}</th>${cols.map((k) => `<td>${fn(k)}</td>`).join('')}</tr>`;
  const ok = (b) => (b ? '<span class="c-high">ok</span>' : '<span class="c-low">no</span>');
  const modes = cols.map((k) => `<th${k.mode === map.recommendedMode ? ' class="sel"' : ''}>${k.mode}</th>`).join('');
  $('m-table').innerHTML = [
    `<tr><th>${map.name} (recommended ${map.recommendedMode})</th>${modes}</tr>`,
    row('map angle / speed', (k) => `${fx(k.m.angleDeg, 2)}° @ ${fx(k.m.speed, 3)} m/s`),
    row('map tof', (k) => `${fx(k.m.tof, 3)} s`),
    row('angle margin', (k) => `−${fx(k.m.angleMarginLow, 1)}° / +${fx(k.m.angleMarginHigh, 1)}°`),
    row('speed margin', (k) => `−${fx(k.m.speedMarginLow)} / +${fx(k.m.speedMarginHigh)} m/s`),
    row('H, V', (k) => `${fx(k.H, 3)}, ${fx(k.V, 3)} m/s`),
    row('turret yaw offset', (k) => `${fx(k.c.yawOffsetDeg, 2)}°`, 'out'),
    row('elevation', (k) => `${fx(k.c.elevationDeg, 2)}°`, 'out'),
    row('exit speed', (k) => `${fx(k.c.speed, 3)} m/s`, 'out'),
    row('hood', (k) => `${fx(k.hoodRot, 4)} rot`, 'out'),
    row('flywheel', (k) => `${fx(k.rps, 2)} rotor rps`, 'out'),
    row('map valid', (k) => `${ok(k.m.valid)} ${k.m.valid ? '' : k.m.reason}`),
    row('|vt| ≤ vt max', (k) => `${ok(Math.abs(vt) <= k.m.vtMax)} (${fx(k.m.vtMax)} m/s)`),
    row('elevation ≥ hood min', (k) => `${ok(k.c.elevationDeg >= hood.min - 1e-6)} (${fx(hood.min, 1)}°)`),
    row('elevation ≤ hood max + margin', (k) =>
      `${ok(k.c.elevationDeg <= hood.max + k.m.angleMarginLow + 1e-6)} (${fx(hood.max, 1)}° + ${fx(k.m.angleMarginLow, 1)}°)`,
    ),
    row('rps ≤ max', (k) => `${ok(k.rps <= mech.flywheel.rotorRpsMax + 1e-6)} (${mech.flywheel.rotorRpsMax})`),
    row('shot valid', (k) =>
      k.reasons.length ? `<span class="c-low">${k.reasons.join(', ')}</span>` : '<span class="c-high">VALID</span>',
    ),
  ].join('');
  const shown = cols.find((k) => k.mode === map.recommendedMode) ?? cols[0];
  drawVectors($('m-vectors'), shown.H, vr, vt);
}

// ---------------------------------------------------------------- 5 Calibrate

function runCal() {
  const rows = parseTunedCsv($('c-csv').value);
  if (rows.length < 2) {
    $('c-result').innerHTML = '<div class="bad">Need at least two rows of d,rps,hoodRot[,tof].</div>';
    return;
  }
  $('c-result').textContent = 'fitting…';
  setTimeout(() => {
    const useTof = $('c-tof').checked;
    const res = calibrate(state.configs.hub, state.mech, rows, { useTof });
    state.calib = res;
    $('c-apply').disabled = false;
    $('c-result').innerHTML = `
      <div><b class="c-chosen">θ at hood 0 (theta0Deg):</b> ${fx(res.theta0Deg)}°</div>
      <div><b class="c-speed">Exit speed per rotor rps:</b> ${fx(res.exitSpeedPerRotorRps, 4)} m/s/rps</div>
      <div><b>Cost:</b> ${fx(res.cost)} (${useTof ? 'distance + TOF' : 'distance only'}, ${rows.length} shots)</div>`;
    $('c-table').innerHTML =
      head(['d', 'rps', 'hood', 'angle', 'speed', 'crossing', 'miss', 'tof model / table', 'scores']) +
      res.rows
        .map((r) =>
          tr([
            fx(r.d, 3),
            r.rps,
            fx(r.hoodRot),
            `${fx(r.angleDeg, 1)}°`,
            fx(r.speed),
            fx(r.xCross),
            `${r.miss >= 0 ? '+' : ''}${fx(r.miss)}`,
            `${fx(r.tofModel)} / ${r.tof ?? '–'}`,
            r.scores ? '<span class="c-high">yes</span>' : `<span class="c-low">${r.reason}</span>`,
          ]),
        )
        .join('');
    plot(
      $('c-profile'),
      [
        {
          type: 'scatter',
          mode: 'lines+markers',
          x: res.profile.map((p) => p.theta0Deg),
          y: res.profile.map((p) => p.cost),
          line: { color: COLORS.chosen },
        },
        {
          type: 'scatter',
          mode: 'markers',
          x: [res.theta0Deg],
          y: [res.cost],
          marker: { symbol: 'star', size: 14, color: '#ffffff' },
        },
      ],
      layout({
        title: { text: 'Cost profile (best exit-speed ratio per θ0)', font: { size: 13 } },
        xaxis: { title: 'θ at hood 0 (deg)' },
        yaxis: { title: 'cost', type: 'log' },
      }),
    );
  }, 20);
}

function applyCal() {
  const res = state.calib;
  state.mech.hood.theta0Deg = Math.round(res.theta0Deg * 100) / 100;
  state.mech.flywheel.exitSpeedPerRotorRps = Math.round(res.exitSpeedPerRotorRps * 10000) / 10000;
  // The fit is the whole exit-speed ratio, so the trim scale goes back to 1.
  state.mech.flywheel.exitSpeedScale = 1;
  refreshForm();
  configEdited();
  $('c-result').insertAdjacentHTML(
    'beforeend',
    '<div class="c-high">Applied to the mechanism fields (exitSpeedScale reset to 1).</div>',
  );
}

// ---------------------------------------------------------------- 6 Feed

/** Feed configs ("feed" and "net feed") whose crossing rule accepts the line from (x, y) to their target. */
function feedMaps(side, x, y) {
  return ['feed', 'netfeed']
    .map((kind) => state.configs[`${kind}_${side}`])
    .filter((c) => {
      const boxes = crossingBoxes(c.feed, c.physics.ballDiameter / 2);
      if (boxes.length === 0) return true;
      const crosses = boxes.some((o) => crossesBox(x, y, c.target[0], c.target[1], o));
      return c.feed.crossing.mode === 'avoid' ? !crosses : crosses;
    });
}

let feedSeq = 0;

function crossingRule(c) {
  const avoid = c.feed.crossing.mode === 'avoid';
  return `line ${avoid ? 'misses' : 'crosses'} ${c.feed.crossing.obstacles.join(avoid ? ' and ' : ' or ')}`;
}

function renderFeed(final = true) {
  const side = $('p-side').value;
  const base = state.configs[`feed_${side}`];
  const [tx, ty] = base.target;
  const [rx, ry] = state.robot;
  const d = Math.hypot(tx - rx, ty - ry);
  const maps = feedMaps(side, rx, ry);
  const crossing = new Set(
    ['feed', 'netfeed'].flatMap((k) => state.configs[`${k}_${side}`].feed.crossing?.obstacles ?? []),
  );
  const Y = (y) => FIELD.h - y;
  const svg = $('p-field');
  svg.innerHTML = '';
  svg.append(svgEl('rect', { x: 0, y: 0, width: FIELD.w, height: FIELD.h, class: 'field' }));
  svg.append(svgEl('line', { x1: FIELD.w / 2, y1: 0, x2: FIELD.w / 2, y2: FIELD.h, class: 'midline' }));
  const reg = base.feed.launchRegion;
  svg.append(
    svgEl('rect', {
      x: reg.x[0],
      y: Y(reg.y[1]),
      width: reg.x[1] - reg.x[0],
      height: reg.y[1] - reg.y[0],
      class: 'region',
    }),
  );
  for (const o of obstacleBoxes(base.feed)) {
    const cls = crossing.has(o.name) ? 'obstacle crossing' : 'obstacle';
    if (o.x[0] === o.x[1] || o.y[0] === o.y[1])
      svg.append(svgEl('line', { x1: o.x[0], y1: Y(o.y[0]), x2: o.x[1], y2: Y(o.y[1]), class: `${cls} plane` }));
    else
      svg.append(
        svgEl('rect', { x: o.x[0], y: Y(o.y[1]), width: o.x[1] - o.x[0], height: o.y[1] - o.y[0], class: cls }),
      );
    const lx = (o.x[0] + o.x[1]) / 2;
    const ly = Y((o.y[0] + o.y[1]) / 2);
    // Planes (zero thickness) get their label alongside, turned to follow the plane.
    const turn = o.x[0] === o.x[1] ? { x: lx + 0.2, transform: `rotate(-90 ${lx + 0.2} ${ly})` } : { x: lx };
    svg.append(svgEl('text', { ...turn, y: ly, class: 'label' }, o.name));
  }
  for (const key of ['left', 'right']) {
    const [x, y] = state.configs[`feed_${key}`].target;
    const sel = key === side;
    svg.append(svgEl('circle', { cx: x, cy: Y(y), r: base.feed.zoneRadius, class: sel ? 'zone sel' : 'zone' }));
    svg.append(svgEl('text', { x, y: Y(y) - base.feed.zoneRadius - 0.12, class: 'label' }, `target ${key}`));
  }
  const color = maps.length === 0 ? COLORS.low : maps[0].name.startsWith('netfeed') ? COLORS.angle : COLORS.high;
  const shown = maps[0];
  if (shown) {
    // Launch points the map's worst-case obstacle profile was built from, at this distance.
    const prof = feedProfile(shown.feed, shown.target, d, shown.physics.ballDiameter / 2);
    for (const b of prof.bearings) {
      svg.append(
        svgEl('circle', {
          cx: tx + d * Math.cos(b * DEG),
          cy: Y(ty + d * Math.sin(b * DEG)),
          r: 0.07,
          fill: color,
          opacity: 0.6,
        }),
      );
    }
  }
  svg.append(svgEl('line', { x1: rx, y1: Y(ry), x2: tx, y2: Y(ty), stroke: color, 'stroke-width': 0.06 }));
  svg.append(svgEl('rect', { x: rx - 0.4, y: Y(ry) - 0.4, width: 0.8, height: 0.8, class: 'robot' }));
  if (!final) return;

  const vr = num('p-vr');
  const out = $('p-readout');
  const where = `<div><b>Robot (turret) at</b> (${fx(rx)}, ${fx(ry)}) m · <b>d</b> = ${fx(d)} m to target ${side}</div>`;
  if (!shown) {
    const rules = ['feed', 'netfeed'].map((k) => `${k}: ${crossingRule(state.configs[`${k}_${side}`])}`).join('; ');
    out.innerHTML = `${where}<div class="c-low">No map covers this line (${rules}).</div>`;
    return;
  }
  const warn = [];
  if (rx < reg.x[0] || rx > reg.x[1] || ry < reg.y[0] || ry > reg.y[1])
    warn.push('outside the launch region the map was built for');
  if (obstacleBoxes(shown.feed).some((o) => rx >= o.x[0] && rx <= o.x[1] && ry >= o.y[0] && ry <= o.y[1]))
    warn.push('inside an obstacle footprint');
  const ctx = makeContext(shown, state.mech);
  const p = solvePoint(ctx, shown, state.mech, d, vr, feedProfile(shown.feed, shown.target, d, ctx.aero.r));
  const live = p.valid
    ? `${fx(p.angleDeg, 1)}° @ ${fx(p.speed)} m/s · hood ${fx(hoodRotForAngle(state.mech, p.angleDeg), 3)} rot · ` +
      `${fx(rpsForExitSpeed(state.mech, p.speed), 1)} rotor rps · tof ${fx(p.tof)} s · score ${fx(p.score)} σ`
    : `<span class="c-low">no shot (${p.reason})</span>`;
  out.innerHTML =
    where +
    `<div><b>Map:</b> <span style="color:${color}">${shown.name}</span>` +
    (shown.feed.crossing ? ` (${crossingRule(shown)})` : '') +
    '</div>' +
    (warn.length ? `<div class="c-speed">Robot is ${warn.join(' and ')}.</div>` : '') +
    `<div><b>Live solve</b> (vr ${vr} m/s): ${live}</div><div id="p-committed" class="hint">loading committed map…</div>`;
  const seq = ++feedSeq;
  loadMap(shown.name)
    .then((map) => {
      if (seq !== feedSeq) return;
      const m = evalMap(map, d, vr, map.recommendedMode);
      $('p-committed').innerHTML =
        `<b>Committed ${map.name}.json</b> (${map.recommendedMode}): ` +
        `${fx(m.angleDeg, 1)}° @ ${fx(m.speed)} m/s · tof ${fx(m.tof)} s · ` +
        (m.valid ? '<span class="c-high">valid</span>' : `<span class="c-low">${m.reason}</span>`);
    })
    .catch((err) => {
      if (seq === feedSeq) $('p-committed').textContent = `Committed map unavailable (${err.message}).`;
    });
}

function initFeedField() {
  const svg = $('p-field');
  svg.setAttribute('viewBox', `-0.4 -0.4 ${FIELD.w + 0.8} ${FIELD.h + 0.8}`);
  const at = (ev) => {
    const pt = svg.createSVGPoint();
    pt.x = ev.clientX;
    pt.y = ev.clientY;
    const p = pt.matrixTransform(svg.getScreenCTM().inverse());
    return [Math.min(Math.max(p.x, 0), FIELD.w), Math.min(Math.max(FIELD.h - p.y, 0), FIELD.h)];
  };
  let dragging = false;
  svg.addEventListener('pointerdown', (ev) => {
    dragging = true;
    svg.setPointerCapture(ev.pointerId);
    state.robot = at(ev);
    renderFeed(false);
  });
  svg.addEventListener('pointermove', (ev) => {
    if (!dragging) return;
    state.robot = at(ev);
    renderFeed(false);
  });
  svg.addEventListener('pointerup', (ev) => {
    if (!dragging) return;
    dragging = false;
    state.robot = at(ev);
    renderFeed();
  });
}

// ---------------------------------------------------------------- 7 Export

function download(name, value) {
  const text = JSON.stringify(value, null, 1) + '\n';
  const url = URL.createObjectURL(new Blob([text], { type: 'application/json' }));
  const a = document.createElement('a');
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  // Sandboxed viewers (e.g. a shared claude.ai link) silently ignore downloads, so always offer a copy too.
  $('x-name').textContent = name;
  $('x-out').value = text;
  $('x-copied').textContent = '';
  $('x-last').hidden = false;
}

async function copyExport() {
  const out = $('x-out');
  try {
    await navigator.clipboard.writeText(out.value);
    $('x-copied').textContent = 'Copied.';
  } catch {
    out.focus();
    out.select();
    $('x-copied').textContent = 'Clipboard blocked: the text is selected, press Ctrl+C.';
  }
}

function renderExport() {
  const sw = state.sweep;
  let map = null;
  try {
    map = sw && sweepMap();
  } catch (err) {
    // e.g. a fit degree the grid cannot support (singular fit matrix)
    $('x-summary').innerHTML = `<div class="c-low">Cannot build the map: ${err.message}</div>`;
  }
  $('x-map').disabled = !map;
  if (map) {
    const s = map.fitStats;
    $('x-map').textContent = `Download ${map.name}.json`;
    $('x-summary').innerHTML =
      `<div><b>${map.name}</b> (${map.kind}): ${map.validCount}/${map.totalCount} valid · degree ${map.degree} · ` +
      `recommended <b>${map.recommendedMode}</b> · angle max |err| ${fx(s.angleDeg.maxAbs, 3)}° · ` +
      `speed max |err| ${fx(s.speed.maxAbs, 3)} m/s · vt global ${fx(map.tangentialLimit.global)} m/s</div>` +
      (sweepStale()
        ? '<div class="c-speed">The config panel changed after this sweep; the export uses the config the sweep ran with.</div>'
        : '');
  } else {
    $('x-map').textContent = 'Download map JSON';
    if (!sw) $('x-summary').innerHTML = '<p class="hint">Run a full sweep to export its map.</p>';
  }
  $('x-preview').textContent = JSON.stringify(robotCalibration(state.mech, state.configs.hub), null, 1);
}

// ---------------------------------------------------------------- wiring

const RENDER = {
  single: runSingle,
  sweep: renderSweep,
  fit: renderFit,
  move: runMove,
  cal: () => state.calib || runCal(),
  feed: () => renderFeed(),
  export: renderExport,
};

function showTab(tab) {
  state.tab = tab;
  for (const b of document.querySelectorAll('#tabs button')) b.classList.toggle('active', b.dataset.tab === tab);
  for (const s of document.querySelectorAll('section.tab')) s.hidden = s.id !== `tab-${tab}`;
  RENDER[tab]();
}

function selectTarget(name) {
  state.name = name;
  refreshForm();
  $('s-d').value = current().kind === 'hub' ? 3 : 7;
  if (state.tab === 'single') runSingle();
}

async function init() {
  window.addEventListener('error', (e) => banner(`Error: ${e.message}`));
  window.addEventListener('unhandledrejection', (e) => banner(`Error: ${e.reason?.message ?? e.reason}`));
  if (!hasPlotly())
    banner(
      'plotly.js did not load (offline or CDN blocked): charts are disabled, every number below still works.',
      'warn',
    );
  try {
    state.mechFile = stripDocs(await fetchJson('configs/mechanism.json'));
    for (const file of MAP_CONFIGS) state.files[file] = await fetchJson(`configs/${file}`);
  } catch (err) {
    banner(`Could not load the configs (${err.message}). Start the UI with: node tools/shotsim/serve.mjs`);
    return;
  }
  resetConfigs();

  const names = Object.keys(state.configs);
  $('target').innerHTML = names.map((n) => `<option>${n}</option>`).join('');
  $('m-source').innerHTML =
    '<option value="sweep">last full sweep</option>' +
    names.map((n) => `<option value="${n}">committed ${n}.json</option>`).join('');
  $('m-source').value = 'hub';
  refreshForm = buildForm($('form'), () => ({ config: current(), mech: state.mech }), configEdited);
  refreshForm();

  $('target').addEventListener('change', (e) => selectTarget(e.target.value));
  $('practiceApex').addEventListener('change', () => {
    applyPracticeApex();
    refreshForm();
    configEdited();
  });
  $('resetConfig').addEventListener('click', () => {
    resetConfigs();
    refreshForm();
    configEdited();
  });
  for (const b of document.querySelectorAll('#tabs button')) b.addEventListener('click', () => showTab(b.dataset.tab));
  $('s-run').addEventListener('click', runSingle);
  for (const id of ['s-d', 's-vr']) $(id).addEventListener('change', runSingle);
  $('g-run').addEventListener('click', runSweep);
  $('g-cancel').addEventListener('click', () => state.job?.cancel());
  $('f-degree').addEventListener('change', renderFit);
  $('m-run').addEventListener('click', runMove);
  for (const id of ['m-d', 'm-vr', 'm-vt', 'm-source']) $(id).addEventListener('change', runMove);
  $('c-run').addEventListener('click', runCal);
  $('c-apply').addEventListener('click', applyCal);
  $('p-side').addEventListener('change', () => renderFeed());
  $('p-vr').addEventListener('change', () => renderFeed());
  $('x-map').addEventListener('click', () => {
    const map = sweepMap();
    download(`${map.name}.json`, map);
  });
  $('x-cal').addEventListener('click', () =>
    download('shooter_calibration.json', robotCalibration(state.mech, state.configs.hub)),
  );
  $('x-copy').addEventListener('click', copyExport);
  initFeedField();
  updateStale();

  try {
    const r = await fetch('data/tuned_hub.csv');
    if (r.ok) $('c-csv').value = await r.text();
  } catch {
    // leave the textarea empty; the team can paste their own table
  }
  showTab('single');
  document.body.dataset.ready = 'true';
}

init();
