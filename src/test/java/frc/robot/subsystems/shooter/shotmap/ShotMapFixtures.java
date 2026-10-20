package frc.robot.subsystems.shooter.shotmap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.nio.file.Path;

/** Parity fixtures written by "node tools/shotsim/cli.mjs fixtures" (src/test/resources/shotmap). */
final class ShotMapFixtures {
  static final double TOL = 1e-6;

  private ShotMapFixtures() {}

  static Path path(String name) throws Exception {
    return Path.of(ShotMapFixtures.class.getResource("/shotmap/" + name).toURI());
  }

  static ShotMap hubMap() throws Exception {
    return ShotMap.load(path("fixture_hub.json"));
  }

  static JsonNode expected() throws Exception {
    return new ObjectMapper().readTree(path("fixture_expected.json").toFile());
  }
}
