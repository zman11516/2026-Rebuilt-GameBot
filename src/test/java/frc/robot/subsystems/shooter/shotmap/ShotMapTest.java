package frc.robot.subsystems.shooter.shotmap;

import static frc.robot.subsystems.shooter.shotmap.ShotMapFixtures.TOL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.NullNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ShotMapTest {
  @Test
  void evalMatchesJsReference() throws Exception {
    ShotMap map = ShotMapFixtures.hubMap();
    JsonNode evals = ShotMapFixtures.expected().get("evals");
    assertTrue(evals.size() > 0);
    for (JsonNode e : evals) {
      double d = e.get("d").asDouble();
      double vr = e.get("vr").asDouble();
      String mode = e.get("mode").asText();
      String at = mode + " d=" + d + " vr=" + vr + " ";
      JsonNode x = e.get("expected");
      ShotMapOutput out = map.eval(d, vr, ShotMap.Mode.valueOf(mode));
      assertEquals(x.get("valid").asBoolean(), out.valid(), at + "valid");
      assertEquals(x.get("reason").asText(), out.reason(), at + "reason");
      assertNear(x, "angleDeg", out.angleDeg(), at);
      assertNear(x, "speed", out.speed(), at);
      assertNear(x, "tof", out.tof(), at);
      assertNear(x, "angleMarginLow", out.angleMarginLow(), at);
      assertNear(x, "angleMarginHigh", out.angleMarginHigh(), at);
      assertNear(x, "speedMarginLow", out.speedMarginLow(), at);
      assertNear(x, "speedMarginHigh", out.speedMarginHigh(), at);
      assertNear(x, "vtMax", out.vtMax(), at);
    }
  }

  @Test
  void hubMapHeader() throws Exception {
    ShotMap map = ShotMapFixtures.hubMap();
    assertEquals("fixture_hub", map.name());
    assertEquals("hub", map.kind());
    assertEquals(ShotMap.Mode.GRID, map.recommendedMode());
    assertTrue(map.target().isEmpty());
    assertFalse(map.lineCrossesBoxes(new Translation2d(2.0, 4.04), new Translation2d(4.62, 4.04)));
  }

  @Test
  void gridFallsBackToPolyWhenACornerIsInvalid() throws Exception {
    ObjectNode node = fixtureTree();
    // Knock out the angle at grid point (d = 3.5, vr = 0), a corner of the cell around (3.4, 0.1)
    ((ArrayNode) node.get("rawGrid").get("angleDeg").get(2)).set(4, NullNode.getInstance());
    ShotMap holed = ShotMap.fromJson(node);
    ShotMap map = ShotMapFixtures.hubMap();

    ShotMapOutput out = holed.eval(3.4, 0.1, ShotMap.Mode.GRID);
    assertFalse(out.valid());
    assertEquals("GRID_CELL_INVALID", out.reason());
    assertEquals(map.eval(3.4, 0.1, ShotMap.Mode.POLY).angleDeg(), out.angleDeg(), TOL);
    assertEquals(map.eval(3.4, 0.1, ShotMap.Mode.GRID).speed(), out.speed(), TOL);
    assertTrue(holed.eval(3.4, 0.1, ShotMap.Mode.POLY).valid());
  }

  @Test
  void feedMapTargetAndCrossingBoxes() throws Exception {
    ObjectNode node = fixtureTree();
    node.put("kind", "feed");
    node.putArray("target").add(2.5).add(6.0);
    ArrayNode boxes = node.putObject("crossing").put("mode", "avoid").putArray("boxes");
    addBox(boxes, "hubBody", 4.0287, 5.2225, 3.4376, 4.6314);
    addBox(boxes, "hubNet", 5.4831, 5.4831, 3.2927, 4.7763); // vertical plane: zero thickness
    ShotMap map = ShotMap.fromJson(node);

    assertEquals("feed", map.kind());
    assertEquals(new Translation2d(2.5, 6.0), map.target().orElseThrow());
    // Straight through the hub body
    assertTrue(map.lineCrossesBoxes(new Translation2d(2.0, 4.0), new Translation2d(8.0, 4.0)));
    // Through the net plane only, then stopping just short of it
    assertTrue(map.lineCrossesBoxes(new Translation2d(5.3, 3.35), new Translation2d(6.0, 3.35)));
    assertFalse(map.lineCrossesBoxes(new Translation2d(5.3, 3.35), new Translation2d(5.4, 3.35)));
    // Well clear of both
    assertFalse(map.lineCrossesBoxes(new Translation2d(2.0, 6.5), new Translation2d(8.0, 7.0)));
  }

  private static ObjectNode fixtureTree() throws Exception {
    return (ObjectNode)
        new ObjectMapper().readTree(ShotMapFixtures.path("fixture_hub.json").toFile());
  }

  private static void addBox(
      ArrayNode boxes, String name, double x0, double x1, double y0, double y1) {
    ObjectNode b = boxes.addObject().put("name", name);
    b.putArray("x").add(x0).add(x1);
    b.putArray("y").add(y0).add(y1);
  }

  private static void assertNear(JsonNode expected, String field, double actual, String at) {
    assertEquals(expected.get(field).asDouble(), actual, TOL, at + field);
  }
}
