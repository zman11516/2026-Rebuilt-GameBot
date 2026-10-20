package frc.robot.subsystems.shooter.shotmap;

import static frc.robot.subsystems.shooter.shotmap.ShotMapFixtures.TOL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShootOnMoveSolverTest {
  @Test
  void solveMatchesJsReference() throws Exception {
    ShotMap map = ShotMapFixtures.hubMap();
    JsonNode fixtures = ShotMapFixtures.expected();
    ShooterCalibration cal = ShooterCalibration.fromJson(fixtures.get("mechanism"));
    JsonNode solves = fixtures.get("solves");
    assertTrue(solves.size() > 0);
    for (JsonNode s : solves) {
      JsonNode st = s.get("state");
      Pose2d pose =
          new Pose2d(
              st.get("x").asDouble(),
              st.get("y").asDouble(),
              new Rotation2d(st.get("heading").asDouble()));
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              st.get("vx").asDouble(), st.get("vy").asDouble(), st.get("omega").asDouble());
      Translation2d target =
          new Translation2d(s.get("target").get(0).asDouble(), s.get("target").get(1).asDouble());
      String mode = s.get("mode").asText();
      SotmSolution r =
          ShootOnMoveSolver.solve(
              pose, speeds, target, map, ShotMap.Mode.valueOf(mode), cal, s.get("latency").asDouble());

      String at = s.get("name").asText() + " " + mode + " ";
      JsonNode x = s.get("expected");
      List<String> reasons = new ArrayList<>();
      x.get("reasons").forEach(n -> reasons.add(n.asText()));
      assertEquals(x.get("valid").asBoolean(), r.valid(), at + "valid");
      assertEquals(reasons, r.reasons(), at + "reasons");
      assertNear(x, "d", r.distance(), at);
      assertNear(x, "vr", r.radialVelocity(), at);
      assertNear(x, "vt", r.tangentialVelocity(), at);
      assertNear(x, "turretYawDeg", r.turretYawDeg(), at);
      assertNear(x, "elevationDeg", r.elevationDeg(), at);
      assertNear(x, "exitSpeed", r.exitSpeed(), at);
      assertNear(x, "hoodRot", r.hoodRot(), at);
      assertNear(x, "flywheelRps", r.flywheelRps(), at);
      assertNear(x, "tof", r.tof(), at);
    }
  }

  @Test
  void compensatesLike4414WorkedExample() {
    // H = 3, V = 6 m/s map shot with the robot sliding left at 2 m/s
    ShootOnMoveSolver.Compensation c = ShootOnMoveSolver.compensate(3, 6, 2);
    assertEquals(-33.69, c.yawOffsetDeg(), 0.01);
    assertEquals(59.0, c.elevationDeg(), 0.05);
    assertEquals(7.0, c.speed(), 1e-9);
  }

  @Test
  void ballGroundVelocityPointsAtTarget() throws Exception {
    ShotMap map = ShotMapFixtures.hubMap();
    ShooterCalibration cal =
        ShooterCalibration.fromJson(ShotMapFixtures.expected().get("mechanism"));
    Translation2d target = new Translation2d(4.62, 4.04);
    // Strafing left at 1 m/s without turning: the turret moves at (0, 1) in the field frame
    SotmSolution r =
        ShootOnMoveSolver.solve(
            new Pose2d(2.0, 4.04, new Rotation2d()),
            new ChassisSpeeds(0, 1, 0),
            target,
            map,
            ShotMap.Mode.POLY,
            cal,
            0.05);
    Translation2d toTarget = target.minus(r.turretPosition());
    double gx = r.launchVector().getX();
    double gy = r.launchVector().getY() + 1.0;
    assertEquals(0, gx * toTarget.getY() - gy * toTarget.getX(), 1e-9);
    assertTrue(gx * toTarget.getX() + gy * toTarget.getY() > 0);
  }

  @Test
  void wrapDegMatchesJs() {
    assertEquals(180, ShootOnMoveSolver.wrapDeg(-180), 0);
    assertEquals(180, ShootOnMoveSolver.wrapDeg(180), 0);
    assertEquals(-170, ShootOnMoveSolver.wrapDeg(190), 1e-12);
    assertEquals(10, ShootOnMoveSolver.wrapDeg(-350), 1e-12);
    assertEquals(180, ShootOnMoveSolver.wrapDeg(540), 0);
  }

  private static void assertNear(JsonNode expected, String field, double actual, String at) {
    assertEquals(expected.get(field).asDouble(), actual, TOL, at + field);
  }
}
