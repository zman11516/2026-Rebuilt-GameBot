// Config panel for the web UI: which fields are editable and how they bind to the config objects.
// `on` picks the object a group edits: the selected target's config or the shared mechanism.

const num = (path, label, unit = '') => ({ path, label, unit });
const opt = (path, label, unit = '') => ({ path, label, unit, nullable: true });
// Steps and sigmas must be > 0 (a zero step never ends the sweep; a zero sigma divides by zero).
const pos = (path, label, unit = '') => ({ path, label, unit, positive: true });
const bool = (path, label) => ({ path, label, bool: true });

export const FIELD_GROUPS = [
  {
    title: 'Physics',
    on: 'config',
    fields: [
      num('physics.dragCoefficient', 'Cd'),
      num('physics.lift.kL', 'kL (Cl per spin)'),
      num('physics.lift.clMax', 'Cl max'),
      num('physics.lift.spinRatio', 'spin ratio'),
      num('physics.airDensity', 'air density', 'kg/m³'),
      num('physics.ballMass', 'ball mass', 'kg'),
      num('physics.ballDiameter', 'ball diameter', 'm'),
      bool('physics.buoyancy', 'buoyancy'),
    ],
  },
  {
    title: 'Mechanism (all targets)',
    on: 'mech',
    fields: [
      num('hood.theta0Deg', 'θ at hood 0', 'deg'),
      num('hood.degPerRot', 'hood slope', 'deg/rot'),
      num('hood.rotMax', 'hood max', 'rot'),
      num('flywheel.exitSpeedPerRotorRps', 'exit speed per rps', 'm/s/rps'),
      num('flywheel.exitSpeedScale', 'exit speed scale'),
      num('flywheel.rotorRpsMax', 'rotor max', 'rps'),
      num('launcher.axleHeight', 'axle height', 'm'),
      num('launcher.axleForward', 'axle forward', 'm'),
      num('launcher.exitRadius', 'exit radius', 'm'),
    ],
  },
  {
    title: 'Hub',
    on: 'config',
    kind: 'hub',
    fields: [
      num('hub.entryHeight', 'entry height', 'm'),
      num('hub.halfWidth', 'opening half-width', 'm'),
      num('hub.windowInset', 'window inset', 'm'),
      num('hub.faceOffset', 'front face offset', 'm'),
      num('hub.lipClearance', 'lip clearance', 'm'),
      num('hub.apexMax', 'apex max', 'm'),
    ],
  },
  {
    title: 'Feed',
    on: 'config',
    kind: 'feed',
    fields: [
      num('feed.landingHeight', 'landing height', 'm'),
      num('feed.zoneRadius', 'zone radius', 'm'),
      num('feed.obstacleClearance', 'obstacle clearance', 'm'),
      num('feed.apexMax', 'apex max', 'm'),
      num('feed.bearingSamples', 'bearing samples'),
      num('feed.bearingSpanDeg', 'bearing span', 'deg'),
    ],
  },
  {
    title: 'Selection',
    on: 'config',
    fields: [
      pos('selection.sigmaSpeed', 'σ speed', 'm/s'),
      pos('selection.sigmaAngleDeg', 'σ angle', 'deg'),
      num('selection.speedBias', 'speed bias'),
      pos('selection.angleStepDeg', 'angle step', 'deg'),
      num('selection.anglePadDeg', 'angle pad', 'deg'),
      opt('selection.fixedHoodRot', 'fixed hood (blank = free)', 'rot'),
    ],
  },
  {
    title: 'Grid',
    on: 'config',
    fields: [
      num('grid.distance.min', 'd min', 'm'),
      num('grid.distance.max', 'd max', 'm'),
      pos('grid.distance.step', 'd step', 'm'),
      num('grid.radialVelocity.min', 'vr min', 'm/s'),
      num('grid.radialVelocity.max', 'vr max', 'm/s'),
      pos('grid.radialVelocity.step', 'vr step', 'm/s'),
    ],
  },
  {
    title: 'Tangential limit',
    on: 'config',
    fields: [
      num('tangential.absMax', '|vt| max', 'm/s'),
      opt('tangential.maxBearingRateRadPerSec', 'bearing rate (blank = off)', 'rad/s'),
    ],
  },
];

export function getPath(obj, path) {
  return path.split('.').reduce((o, k) => (o === undefined || o === null ? undefined : o[k]), obj);
}

export function setPath(obj, path, value) {
  const keys = path.split('.');
  const last = keys.pop();
  const parent = keys.reduce((o, k) => (o[k] ??= {}), obj);
  parent[last] = value;
}

/**
 * Build the panel inside `root`. objects() returns { config, mech } (the live, editable objects);
 * onChange(group, field) runs after every accepted edit. Returns refresh(), which reloads every
 * input from the objects and shows only the groups for the selected config's kind.
 */
export function buildForm(root, objects, onChange) {
  const inputs = [];
  const sets = [];
  for (const group of FIELD_GROUPS) {
    const set = document.createElement('fieldset');
    set.innerHTML = `<legend>${group.title}</legend>`;
    for (const field of group.fields) {
      const label = document.createElement('label');
      const input = document.createElement('input');
      input.dataset.path = field.path;
      if (field.bool) input.type = 'checkbox';
      else {
        input.type = 'number';
        input.step = 'any';
      }
      label.innerHTML = `<span>${field.label}${field.unit ? ` <i>${field.unit}</i>` : ''}</span>`;
      label.append(input);
      set.append(label);
      input.addEventListener('change', () => {
        const target = objects()[group.on];
        const blank = input.value.trim() === '';
        let value;
        if (field.bool) value = input.checked;
        else if (blank)
          value = field.nullable ? null : NaN; // Number('') would be 0
        else value = Number(input.value);
        if (!field.bool && value !== null && (!Number.isFinite(value) || (field.positive && value <= 0))) {
          input.classList.add('bad');
          return;
        }
        input.classList.remove('bad');
        setPath(target, field.path, value);
        onChange(group, field);
      });
      inputs.push({ group, field, input });
    }
    root.append(set);
    sets.push({ group, set });
  }
  return function refresh() {
    const objs = objects();
    for (const { group, set } of sets) set.hidden = group.kind !== undefined && group.kind !== objs.config.kind;
    for (const { group, field, input } of inputs) {
      if (group.kind !== undefined && group.kind !== objs.config.kind) continue;
      const v = getPath(objs[group.on], field.path);
      if (field.bool) input.checked = !!v;
      else input.value = v === null || v === undefined ? '' : String(v);
      input.classList.remove('bad');
    }
  };
}
