// Plotly helpers for the web UI: one dark theme, and a text fallback when the CDN script is missing.

export const COLORS = {
  bg: '#1b2232',
  panel: '#222a3d',
  grid: '#33405a',
  text: '#c9d3e6',
  muted: '#7f8aa3',
  low: '#e5534b', // lower speed limit (short)
  high: '#57ab5a', // upper speed limit (long)
  chosen: '#4c8dff',
  speed: '#f0883e',
  angle: '#a371f7',
  sigma: '#e3b341',
  invalid: '#3a4256',
};

export const hasPlotly = () => typeof window !== 'undefined' && typeof window.Plotly !== 'undefined';

/** Themed Plotly axis. */
export const darkAxis = (extra = {}) => ({
  gridcolor: COLORS.grid,
  zerolinecolor: COLORS.grid,
  linecolor: COLORS.grid,
  color: COLORS.text,
  ...extra,
});

/** Base dark layout; `extra.xaxis` / `extra.yaxis` are merged into the themed axes. */
export function layout(extra = {}) {
  const { xaxis, yaxis, ...rest } = extra;
  return {
    paper_bgcolor: COLORS.bg,
    plot_bgcolor: COLORS.bg,
    font: { color: COLORS.text, size: 12 },
    margin: { l: 56, r: 16, t: 36, b: 50 },
    showlegend: false,
    xaxis: darkAxis(xaxis),
    yaxis: darkAxis(yaxis),
    ...rest,
  };
}

/** Draw (or redraw) a plot; without Plotly, leave a note in the element instead. */
export function plot(el, traces, lay, config = {}) {
  if (!hasPlotly()) {
    el.classList.add('noplot');
    el.textContent = 'Chart unavailable: plotly.js did not load (offline or CDN blocked).';
    return;
  }
  window.Plotly.react(el, traces, lay, { responsive: true, displaylogo: false, ...config });
}

/** Line trace through points with optional nulls (gaps). */
export function line(x, y, color, width = 1, extra = {}) {
  return { type: 'scatter', mode: 'lines', x, y, line: { color, width }, hoverinfo: 'skip', ...extra };
}
