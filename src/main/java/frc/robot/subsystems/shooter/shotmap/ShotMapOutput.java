package frc.robot.subsystems.shooter.shotmap;

/**
 * One shot-map lookup. The values are always filled in (from the clamped inputs) so the turret
 * and hood can keep tracking while the shot itself is not valid.
 *
 * @param valid true when (distance, radial velocity) is inside the simulated region
 * @param reason "OK", "RADIAL_OUT_OF_RANGE", "DISTANCE_OUT_OF_RANGE" or "GRID_CELL_INVALID"
 * @param angleDeg launch angle above horizontal (deg)
 * @param speed ball exit speed (m/s)
 * @param tof time of flight (s)
 * @param angleMarginLow how far the angle can drop and still score (deg, >= 0)
 * @param angleMarginHigh how far the angle can rise and still score (deg, >= 0)
 * @param speedMarginLow how far the speed can drop and still score (m/s, >= 0)
 * @param speedMarginHigh how far the speed can rise and still score (m/s, >= 0)
 * @param vtMax largest tangential robot speed the shot tolerates (m/s)
 */
public record ShotMapOutput(
    boolean valid,
    String reason,
    double angleDeg,
    double speed,
    double tof,
    double angleMarginLow,
    double angleMarginHigh,
    double speedMarginLow,
    double speedMarginHigh,
    double vtMax) {}
