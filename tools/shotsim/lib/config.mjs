// Config handling shared by the CLI and the browser (no file IO here).

export const MAP_CONFIGS = ['hub.json', 'feed.json', 'netfeed.json'];

/** Drop documentation keys ("_source", "_doc", ...) so configs hash and diff by content. */
export function stripDocs(v) {
  if (Array.isArray(v)) return v.map(stripDocs);
  if (v && typeof v === 'object') {
    return Object.fromEntries(Object.entries(v).filter(([k]) => !k.startsWith('_')).map(([k, x]) => [k, stripDocs(x)]));
  }
  return v;
}

/**
 * One generated map per (config, target). Hub configs produce one map named after the config;
 * feed configs produce "<name>_<target>" for every entry in "targets".
 */
export function expandConfig(config) {
  const c = stripDocs(config);
  if (c.kind === 'hub') return [c];
  return Object.entries(c.targets).map(([key, xy]) => ({ ...c, name: `${c.name}_${key}`, target: xy }));
}

/** The calibration file the robot loads (src/main/deploy/shooter_calibration.json). */
export function robotCalibration(mech, hubConfig) {
  const m = stripDocs(mech);
  return {
    schema: 'shotsim5454-calibration/1',
    hood: m.hood,
    flywheel: m.flywheel,
    launcher: m.launcher,
    turretOffset: m.turretOffset,
    sotm: stripDocs(hubConfig.sotm),
  };
}
