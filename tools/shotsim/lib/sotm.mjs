// Shoot-on-the-move solver. Reference implementation for
// frc.robot.subsystems.shooter.shotmap.ShootOnMoveSolver.
//
// The map already handles RADIAL velocity (it was simulated). TANGENTIAL velocity is cancelled
// with 3D vectors, as 4414 does: in the hub frame (u toward the target, t left of u, z up)
//   shooter launch vector L = H*u - vt*t + V*z,  with H = s*cos(theta), V = s*sin(theta)
// so L + robot velocity = (H + vr)*u + V*z, exactly the simulated radial shot.

import { evalMap } from './mapeval.mjs';
import { hoodRotForAngle, rpsForExitSpeed, angleRange } from './mechanism.mjs';

const DEG = Math.PI / 180;

/** Turret yaw offset, elevation and speed for cancelling tangential velocity vt. */
export function compensate(H, V, vt) {
  return {
    yawOffsetDeg: Math.atan2(-vt, H) / DEG,
    elevationDeg: Math.atan2(V, Math.hypot(H, vt)) / DEG,
    speed: Math.sqrt(H * H + vt * vt + V * V),
  };
}

/** WPILib Pose2d.exp(Twist2d) for a pose (x, y, heading rad) and robot-relative twist. */
export function poseExp(x, y, heading, dx, dy, dtheta) {
  const s = Math.sin(dtheta);
  const c = Math.cos(dtheta);
  let sinTerm;
  let cosTerm;
  if (Math.abs(dtheta) < 1e-9) {
    sinTerm = 1 - (dtheta * dtheta) / 6;
    cosTerm = 0.5 * dtheta;
  } else {
    sinTerm = s / dtheta;
    cosTerm = (1 - c) / dtheta;
  }
  const tx = dx * sinTerm - dy * cosTerm;
  const ty = dx * cosTerm + dy * sinTerm;
  const ch = Math.cos(heading);
  const sh = Math.sin(heading);
  return { x: x + tx * ch - ty * sh, y: y + tx * sh + ty * ch, heading: heading + dtheta };
}

function wrapDeg(a) {
  let w = ((a + 180) % 360 + 360) % 360 - 180;
  if (w === -180) w = 180;
  return w;
}

/**
 * Solve a moving shot.
 * @param state { x, y, heading (rad), vx, vy, omega } pose and ROBOT-RELATIVE chassis speeds
 * @param target [x, y] field target (already alliance-flipped)
 * @param map exported shot map
 * @param mech mechanism config (hood + flywheel calibration, launcher geometry)
 * @param opts { latency (s), turretOffset [x, y] (m, robot frame), mode 'POLY'|'GRID' }
 */
export function solve(state, target, map, mech, opts) {
  const latency = opts.latency ?? 0;
  const p = poseExp(state.x, state.y, state.heading, state.vx * latency, state.vy * latency, state.omega * latency);
  const [ox, oy] = opts.turretOffset;
  const ch = Math.cos(p.heading);
  const sh = Math.sin(p.heading);
  const tx = p.x + ox * ch - oy * sh;
  const ty = p.y + ox * sh + oy * ch;
  // Turret-point velocity: v + omega x r, robot frame, rotated into the field frame.
  const rvx = state.vx - state.omega * oy;
  const rvy = state.vy + state.omega * ox;
  const fvx = rvx * ch - rvy * sh;
  const fvy = rvx * sh + rvy * ch;

  const dx = target[0] - tx;
  const dy = target[1] - ty;
  const d = Math.hypot(dx, dy);
  const ux = dx / d;
  const uy = dy / d;
  const vr = fvx * ux + fvy * uy; // + closing on the target
  const vt = -fvx * uy + fvy * ux; // + to the left of the line of sight

  const m = evalMap(map, d, vr, opts.mode ?? 'POLY');
  const th = m.angleDeg * DEG;
  const H = m.speed * Math.cos(th);
  const V = m.speed * Math.sin(th);
  const comp = compensate(H, V, vt);

  // Launch vector in the field frame (for logging / checks).
  const Lx = H * ux + vt * uy;
  const Ly = H * uy - vt * ux;
  const fieldYawDeg = Math.atan2(Ly, Lx) / DEG;
  // The release point sits axleLateral to the LEFT of the turret axis (signed); rotate so the
  // offset shot line still passes through the target.
  const lateral = mech.launcher.axleLateral ?? 0;
  const lateralDeg = Math.abs(lateral) < d ? Math.asin(lateral / d) / DEG : 0;
  const turretYawDeg = wrapDeg(fieldYawDeg - p.heading / DEG - lateralDeg);

  const reasons = [];
  // A NaN pose/speed (or standing on the target) would otherwise slip through every comparison.
  const finite = [d, vr, vt, comp.elevationDeg, comp.speed].every(Number.isFinite);
  if (!finite || d < 1e-6) reasons.push('NON_FINITE_INPUT');
  if (!m.valid) reasons.push(m.reason);
  if (Math.abs(vt) > m.vtMax) reasons.push('TANGENTIAL_TOO_FAST');
  const range = angleRange(mech);
  if (comp.elevationDeg < range.min - 1e-6) reasons.push('HOOD_BELOW_MIN');
  // Above the top angle the hood clamps, which still scores while the miss is inside the map's margin.
  if (comp.elevationDeg > range.max + m.angleMarginLow + 1e-6) reasons.push('HOOD_ABOVE_MAX');
  const rps = rpsForExitSpeed(mech, comp.speed);
  if (rps > mech.flywheel.rotorRpsMax + 1e-6) reasons.push('FLYWHEEL_TOO_FAST');

  return {
    valid: reasons.length === 0,
    reasons,
    d,
    vr,
    vt,
    turretPos: [tx, ty],
    map: m,
    launchVector: [Lx, Ly, V],
    fieldYawDeg,
    turretYawDeg,
    elevationDeg: comp.elevationDeg,
    exitSpeed: comp.speed,
    hoodRot: hoodRotForAngle(mech, comp.elevationDeg),
    flywheelRps: rps,
    tof: m.tof,
  };
}
