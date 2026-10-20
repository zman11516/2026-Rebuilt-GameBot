// Module Web Worker for the Full Sweep tab: solves the radial-velocity rows it is given.
//   in:  { config, mech, rows: [ri, ...] }
//   out: { type: 'progress', done } ... then { type: 'done', points } or { type: 'error', message }

import { buildGrid } from './lib/grid.mjs';

self.onmessage = (e) => {
  const { config, mech, rows } = e.data;
  const mine = new Set(rows);
  try {
    const grid = buildGrid(config, mech, {
      rowFilter: (ri) => mine.has(ri),
      onProgress: (done) => self.postMessage({ type: 'progress', done }),
    });
    self.postMessage({ type: 'done', points: grid.points });
  } catch (err) {
    self.postMessage({ type: 'error', message: String(err?.stack ?? err) });
  }
};
