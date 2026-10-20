// 5454 launcher geometry and the two calibration knobs (hood angle offset, exit-speed ratio).
// Mirrors frc.robot.subsystems.shooter.shotmap.ShooterCalibration on the robot.

const DEG = Math.PI / 180;

/** Launch angle (deg above horizontal) for a hood CANcoder reading (rotations). */
export function angleForHoodRot(mech, rot) {
  return mech.hood.theta0Deg + mech.hood.degPerRot * rot;
}

/** Hood CANcoder rotations that produce a launch angle (deg), clamped to the hood travel. */
export function hoodRotForAngle(mech, angleDeg) {
  const rot = (angleDeg - mech.hood.theta0Deg) / mech.hood.degPerRot;
  return Math.min(Math.max(rot, mech.hood.rotMin), mech.hood.rotMax);
}

/** Reachable launch-angle range (deg) over the hood travel. */
export function angleRange(mech) {
  const a = angleForHoodRot(mech, mech.hood.rotMin);
  const b = angleForHoodRot(mech, mech.hood.rotMax);
  return { min: Math.min(a, b), max: Math.max(a, b) };
}

/** Ball exit speed (m/s) for a flywheel rotor speed (rotor rotations per second). */
export function exitSpeedForRps(mech, rps) {
  return mech.flywheel.exitSpeedPerRotorRps * mech.flywheel.exitSpeedScale * rps;
}

/** Flywheel rotor speed (rps) for a ball exit speed (m/s). */
export function rpsForExitSpeed(mech, speed) {
  return speed / (mech.flywheel.exitSpeedPerRotorRps * mech.flywheel.exitSpeedScale);
}

/** Exit-speed range (m/s) reachable by the flywheel. */
export function speedRange(mech) {
  return {
    min: exitSpeedForRps(mech, mech.flywheel.rotorRpsMin),
    max: exitSpeedForRps(mech, mech.flywheel.rotorRpsMax),
  };
}

/**
 * Ball-centre release point for a launch angle, in the shot plane.
 * x is measured from the turret axis along the shot direction, z from the floor.
 * The hood is concentric with the flywheel axle, so the ball leaves tangent to the wheel:
 * release = axle + exitRadius * (-sin(theta), cos(theta)).
 */
export function releasePoint(mech, angleDeg) {
  const l = mech.launcher;
  const th = angleDeg * DEG;
  return {
    x: l.axleForward - l.exitRadius * Math.sin(th),
    z: l.axleHeight + l.exitRadius * Math.cos(th),
  };
}
