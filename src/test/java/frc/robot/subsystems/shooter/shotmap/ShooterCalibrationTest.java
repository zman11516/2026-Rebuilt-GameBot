package frc.robot.subsystems.shooter.shotmap;

import static frc.robot.subsystems.shooter.shotmap.ShotMapFixtures.TOL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.junit.jupiter.api.Test;

class ShooterCalibrationTest {
  private static ShooterCalibration fixtureCalibration() throws Exception {
    return ShooterCalibration.fromJson(ShotMapFixtures.expected().get("mechanism"));
  }

  @Test
  void hoodConversionsMatchJsReference() throws Exception {
    ShooterCalibration cal = fixtureCalibration();
    JsonNode conversions = ShotMapFixtures.expected().get("conversions");
    assertTrue(conversions.size() > 0);
    for (JsonNode c : conversions) {
      // The first key is the input
      if (c.fieldNames().next().equals("hoodRot")) {
        double rot = c.get("hoodRot").asDouble();
        assertEquals(c.get("angleDeg").asDouble(), cal.angleForHoodRot(rot), TOL, "rot " + rot);
      } else {
        double angle = c.get("angleDeg").asDouble();
        assertEquals(c.get("hoodRot").asDouble(), cal.hoodRotForAngle(angle), TOL, "deg " + angle);
      }
    }
  }

  @Test
  void flywheelConversionsMatchJsReference() throws Exception {
    ShooterCalibration cal = fixtureCalibration();
    JsonNode speeds = ShotMapFixtures.expected().get("speeds");
    assertTrue(speeds.size() > 0);
    for (JsonNode s : speeds) {
      if (s.fieldNames().next().equals("exitSpeed")) {
        double mps = s.get("exitSpeed").asDouble();
        assertEquals(s.get("rotorRps").asDouble(), cal.rpsForExitSpeed(mps), TOL, "mps " + mps);
      } else {
        double rps = s.get("rotorRps").asDouble();
        assertEquals(s.get("exitSpeed").asDouble(), cal.exitSpeedForRps(rps), TOL, "rps " + rps);
      }
    }
  }

  @Test
  void hoodClampsToItsTravel() throws Exception {
    ShooterCalibration cal = fixtureCalibration();
    double high = cal.hoodRotForAngle(cal.maxAngleDeg());
    double low = cal.hoodRotForAngle(cal.minAngleDeg());
    assertEquals(high, cal.hoodRotForAngle(cal.maxAngleDeg() + 10), TOL);
    assertEquals(low, cal.hoodRotForAngle(cal.minAngleDeg() - 10), TOL);
    assertEquals(cal.maxAngleDeg(), cal.angleForHoodRot(high), TOL);
    assertEquals(cal.minAngleDeg(), cal.angleForHoodRot(low), TOL);
  }

  @Test
  void trimsShiftTheHoodAndScaleTheFlywheel() throws Exception {
    ShooterCalibration cal = fixtureCalibration();
    double angle = cal.angleForHoodRot(0.1);
    double minAngle = cal.minAngleDeg();
    double scale = cal.exitSpeedScale();
    double speed = cal.exitSpeedForRps(60);

    cal.setHoodOffsetDeg(1.5);
    assertEquals(1.5, cal.hoodOffsetDeg(), 0);
    assertEquals(angle + 1.5, cal.angleForHoodRot(0.1), TOL);
    assertEquals(0.1, cal.hoodRotForAngle(angle + 1.5), TOL);
    assertEquals(minAngle + 1.5, cal.minAngleDeg(), TOL);

    cal.setExitSpeedScale(0.9);
    assertEquals(0.9, cal.exitSpeedScale(), 0);
    assertEquals(speed / scale * 0.9, cal.exitSpeedForRps(60), TOL);
    assertEquals(60, cal.rpsForExitSpeed(speed / scale * 0.9), TOL);
    cal.setExitSpeedScale(0); // a divisor: ignored
    assertEquals(0.9, cal.exitSpeedScale(), 0);
  }

  @Test
  void sotmLatenciesAreOptional() throws Exception {
    JsonNode mechanism = ShotMapFixtures.expected().get("mechanism");
    ShooterCalibration cal = ShooterCalibration.fromJson(mechanism);
    assertEquals(0, cal.phaseDelay(), 0);
    assertEquals(0, cal.mechLatency(), 0);
    assertEquals(mechanism.get("turretOffset").get(0).asDouble(), cal.turretOffset().getX(), 0);
    assertEquals(mechanism.get("turretOffset").get(1).asDouble(), cal.turretOffset().getY(), 0);

    ObjectNode withSotm = mechanism.deepCopy();
    withSotm.putObject("sotm").put("phaseDelay", 0.03).put("mechLatency", 0.02);
    cal = ShooterCalibration.fromJson(withSotm);
    assertEquals(0.03, cal.phaseDelay(), 0);
    assertEquals(0.02, cal.mechLatency(), 0);
  }
}
