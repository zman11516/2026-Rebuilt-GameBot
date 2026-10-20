package frc.robot.subsystems.shooter.shotmap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;

/**
 * Shoot-on-the-move solver (port of tools/shotsim/lib/sotm.mjs).
 *
 * <p>The map already handles RADIAL velocity (it was simulated). TANGENTIAL velocity is cancelled
 * with 3D vectors, as 4414 does: in the target frame (u toward the target, t left of u, z up) the
 * launch vector is L = H*u - vt*t + V*z, with H = s*cos(theta) and V = s*sin(theta), so L plus the
 * robot velocity is (H + vr)*u + V*z, exactly the simulated radial shot.
 */
public final class ShootOnMoveSolver {
  private static final double DEG = Math.PI / 180;

  private ShootOnMoveSolver() {}

  /** Turret yaw offset (deg), elevation (deg) and exit speed (m/s) that cancel vt. */
  public record Compensation(double yawOffsetDeg, double elevationDeg, double speed) {}

  /** Compensate a map shot (horizontal H, vertical V, m/s) for tangential velocity vt (m/s). */
  public static Compensation compensate(double horizontal, double vertical, double vt) {
    return new Compensation(
        Math.atan2(-vt, horizontal) / DEG,
        Math.atan2(vertical, Math.hypot(horizontal, vt)) / DEG,
        Math.sqrt(horizontal * horizontal + vt * vt + vertical * vertical));
  }

  /**
   * Solve a moving shot.
   *
   * @param pose robot pose
   * @param robotRelativeSpeeds ROBOT-RELATIVE chassis speeds
   * @param target field target (already alliance-flipped)
   * @param map shot map for that target
   * @param mode POLY or GRID map evaluation
   * @param cal hood / flywheel calibration and launcher geometry
   * @param latencySec how far ahead to predict the pose (s)
   */
  public static SotmSolution solve(
      Pose2d pose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation2d target,
      ShotMap map,
      ShotMap.Mode mode,
      ShooterCalibration cal,
      double latencySec) {
    double vx = robotRelativeSpeeds.vxMetersPerSecond;
    double vy = robotRelativeSpeeds.vyMetersPerSecond;
    double omega = robotRelativeSpeeds.omegaRadiansPerSecond;
    Pose2d p = pose.exp(new Twist2d(vx * latencySec, vy * latencySec, omega * latencySec));
    Rotation2d heading = p.getRotation();
    Translation2d offset = cal.turretOffset();
    Translation2d turret = p.getTranslation().plus(offset.rotateBy(heading));
    // Turret-point velocity: v + omega x r, robot frame, rotated into the field frame.
    Translation2d turretVel =
        new Translation2d(vx - omega * offset.getY(), vy + omega * offset.getX()).rotateBy(heading);

    double dx = target.getX() - turret.getX();
    double dy = target.getY() - turret.getY();
    double d = Math.hypot(dx, dy);
    double ux = dx / d;
    double uy = dy / d;
    double vr = turretVel.getX() * ux + turretVel.getY() * uy; // + closing on the target
    double vt = -turretVel.getX() * uy + turretVel.getY() * ux; // + to the left of the line of sight

    ShotMapOutput m = map.eval(d, vr, mode);
    double th = m.angleDeg() * DEG;
    double h = m.speed() * Math.cos(th);
    double v = m.speed() * Math.sin(th);
    Compensation comp = compensate(h, v, vt);

    // Launch vector in the field frame (for logging / checks).
    double lx = h * ux + vt * uy;
    double ly = h * uy - vt * ux;
    double fieldYawDeg = Math.atan2(ly, lx) / DEG;
    // The release point sits axleLateral to the LEFT of the turret axis (signed); rotate so the
    // offset shot line still passes through the target.
    double lateral = cal.axleLateral();
    double lateralDeg = Math.abs(lateral) < d ? Math.asin(lateral / d) / DEG : 0;
    double turretYawDeg = wrapDeg(fieldYawDeg - heading.getRadians() / DEG - lateralDeg);

    List<String> reasons = new ArrayList<>();
    if (!m.valid()) reasons.add(m.reason());
    if (Math.abs(vt) > m.vtMax()) reasons.add("TANGENTIAL_TOO_FAST");
    if (comp.elevationDeg() < cal.minAngleDeg() - 1e-6) reasons.add("HOOD_BELOW_MIN");
    double rps = cal.rpsForExitSpeed(comp.speed());
    if (rps > cal.rotorRpsMax() + 1e-6) reasons.add("FLYWHEEL_TOO_FAST");

    return new SotmSolution(
        reasons.isEmpty(),
        List.copyOf(reasons),
        d,
        vr,
        vt,
        turret,
        m,
        fieldYawDeg,
        turretYawDeg,
        comp.elevationDeg(),
        comp.speed(),
        cal.hoodRotForAngle(comp.elevationDeg()),
        rps,
        m.tof(),
        new Translation3d(lx, ly, v));
  }

  /** Wrap to (-180, 180] degrees; -180 maps to 180 (same as sotm.mjs). */
  static double wrapDeg(double a) {
    double w = ((a + 180) % 360 + 360) % 360 - 180;
    return w == -180 ? 180 : w;
  }
}
