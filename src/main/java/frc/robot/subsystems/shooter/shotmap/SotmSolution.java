package frc.robot.subsystems.shooter.shotmap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;

/**
 * Result of {@link ShootOnMoveSolver#solve}.
 *
 * @param valid true when nothing in {@code reasons} blocks the shot
 * @param reasons map reason plus "TANGENTIAL_TOO_FAST", "HOOD_BELOW_MIN", "FLYWHEEL_TOO_FAST"
 * @param distance turret axis to target, 2D (m)
 * @param radialVelocity turret velocity toward the target (m/s, + closing)
 * @param tangentialVelocity turret velocity across the line of sight (m/s, + to the left)
 * @param turretPosition turret axis on the field at release (m)
 * @param map the map lookup at (distance, radialVelocity)
 * @param fieldYawDeg field-relative launch direction (deg, CCW)
 * @param turretYawDeg robot-relative turret command (deg, CCW, wrapped to (-180, 180])
 * @param elevationDeg launch angle the hood must produce (deg)
 * @param exitSpeed ball exit speed relative to the robot (m/s)
 * @param hoodRot hood command (CANcoder rotations, clamped to the hood travel)
 * @param flywheelRps flywheel command (rotor rotations per second, not clamped)
 * @param tof time of flight (s)
 * @param launchVector launch velocity relative to the robot, field frame (m/s)
 */
public record SotmSolution(
    boolean valid,
    List<String> reasons,
    double distance,
    double radialVelocity,
    double tangentialVelocity,
    Translation2d turretPosition,
    ShotMapOutput map,
    double fieldYawDeg,
    double turretYawDeg,
    double elevationDeg,
    double exitSpeed,
    double hoodRot,
    double flywheelRps,
    double tof,
    Translation3d launchVector) {}
