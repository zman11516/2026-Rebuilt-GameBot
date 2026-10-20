package frc.robot.subsystems.shooter.shotmap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.IOException;
import java.nio.file.Path;

/**
 * 5454 launcher geometry and the hood / flywheel conversions, from deploy/shooter_calibration.json
 * (written by tools/shotsim). Port of tools/shotsim/lib/mechanism.mjs.
 *
 * <p>The hood command is CANcoder rotations: launch angle = theta0Deg + degPerRot * rot. The
 * flywheel command is rotor rps: exit speed = exitSpeedPerRotorRps * exitSpeedScale * rps.
 *
 * <p>Like 4414, real shots are matched to the simulation with two live trims: a hood angle offset
 * (added to theta0Deg) and an exit-speed scale (replaces the file's exitSpeedScale).
 */
public final class ShooterCalibration {
  private final double theta0Deg;
  private final double degPerRot;
  private final double rotMin;
  private final double rotMax;
  private final double exitSpeedPerRotorRps;
  private final double rotorRpsMin;
  private final double rotorRpsMax;
  private final double axleLateral;
  private final double axleHeight;
  private final Translation2d turretOffset;
  private final double phaseDelay;
  private final double mechLatency;

  private double hoodOffsetDeg = 0;
  private double exitSpeedScale;

  /** Read deploy/shooter_calibration.json. */
  public static ShooterCalibration load(Path file) throws IOException {
    return fromJson(new ObjectMapper().readTree(file.toFile()));
  }

  /**
   * Build from {hood, flywheel, launcher, turretOffset, sotm?}: the calibration file or the
   * "mechanism" object of a shot map. Without "sotm" both latencies are 0.
   */
  public static ShooterCalibration fromJson(JsonNode node) {
    return new ShooterCalibration(node);
  }

  private ShooterCalibration(JsonNode node) {
    theta0Deg = ShotMap.requireNumber(node, "hood.theta0Deg");
    degPerRot = ShotMap.requireNumber(node, "hood.degPerRot");
    rotMin = ShotMap.requireNumber(node, "hood.rotMin");
    rotMax = ShotMap.requireNumber(node, "hood.rotMax");
    exitSpeedPerRotorRps = ShotMap.requireNumber(node, "flywheel.exitSpeedPerRotorRps");
    exitSpeedScale = ShotMap.requireNumber(node, "flywheel.exitSpeedScale");
    rotorRpsMin = ShotMap.requireNumber(node, "flywheel.rotorRpsMin");
    rotorRpsMax = ShotMap.requireNumber(node, "flywheel.rotorRpsMax");
    axleLateral = node.path("launcher").path("axleLateral").asDouble(0);
    axleHeight = ShotMap.requireNumber(node, "launcher.axleHeight");
    JsonNode offset = node.path("turretOffset");
    if (!offset.isArray() || offset.size() != 2) {
      throw new IllegalArgumentException("missing turretOffset [x, y]");
    }
    turretOffset = new Translation2d(offset.get(0).asDouble(), offset.get(1).asDouble());
    phaseDelay = node.path("sotm").path("phaseDelay").asDouble(0);
    mechLatency = node.path("sotm").path("mechLatency").asDouble(0);
  }

  /** Launch angle (deg above horizontal) for a hood CANcoder reading (rotations). */
  public double angleForHoodRot(double rot) {
    return theta0Deg + hoodOffsetDeg + degPerRot * rot;
  }

  /** Hood CANcoder rotations that produce a launch angle (deg), clamped to the hood travel. */
  public double hoodRotForAngle(double angleDeg) {
    double rot = (angleDeg - (theta0Deg + hoodOffsetDeg)) / degPerRot;
    return Math.min(Math.max(rot, rotMin), rotMax);
  }

  /** Lowest launch angle the hood reaches (deg). */
  public double minAngleDeg() {
    return Math.min(angleForHoodRot(rotMin), angleForHoodRot(rotMax));
  }

  /** Highest launch angle the hood reaches (deg). */
  public double maxAngleDeg() {
    return Math.max(angleForHoodRot(rotMin), angleForHoodRot(rotMax));
  }

  /** Ball exit speed (m/s) for a flywheel rotor speed (rps). */
  public double exitSpeedForRps(double rps) {
    return exitSpeedPerRotorRps * exitSpeedScale * rps;
  }

  /** Flywheel rotor speed (rps) for a ball exit speed (m/s). */
  public double rpsForExitSpeed(double mps) {
    return mps / (exitSpeedPerRotorRps * exitSpeedScale);
  }

  public double rotorRpsMin() {
    return rotorRpsMin;
  }

  public double rotorRpsMax() {
    return rotorRpsMax;
  }

  /** Launch-angle change per hood rotation (deg/rot, negative: more rotation = flatter shot). */
  public double degPerRot() {
    return degPerRot;
  }

  /** Release point offset to the LEFT of the turret axis (m, signed). */
  public double axleLateral() {
    return axleLateral;
  }

  /** Flywheel axle height above the floor (m). */
  public double axleHeight() {
    return axleHeight;
  }

  /** Turret axis in the robot frame (m). */
  public Translation2d turretOffset() {
    return turretOffset;
  }

  /** Control phase delay to look ahead by (s); 6328 uses 30 ms. */
  public double phaseDelay() {
    return phaseDelay;
  }

  /** Feed-to-exit delay of the ball (s). */
  public double mechLatency() {
    return mechLatency;
  }

  /** Trim: degrees added to every launch angle the hood produces. */
  public void setHoodOffsetDeg(double offsetDeg) {
    hoodOffsetDeg = offsetDeg;
  }

  public double hoodOffsetDeg() {
    return hoodOffsetDeg;
  }

  /** Trim: measured / modelled exit speed. Ignored unless positive (it is a divisor). */
  public void setExitSpeedScale(double scale) {
    if (scale > 0) exitSpeedScale = scale;
  }

  public double exitSpeedScale() {
    return exitSpeedScale;
  }
}
