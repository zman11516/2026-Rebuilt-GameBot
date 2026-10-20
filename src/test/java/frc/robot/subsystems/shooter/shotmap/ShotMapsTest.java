package frc.robot.subsystems.shooter.shotmap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ShotMapsTest {
  @Test
  void deployedMapsLoad() {
    ShotMaps maps = new ShotMaps(Path.of("src/main/deploy"));
    assertEquals("", maps.loadErrors());
    assertTrue(maps.available());
    assertEquals("hub", maps.hub().kind());
    for (boolean left : new boolean[] {true, false}) {
      for (boolean overNet : new boolean[] {true, false}) {
        ShotMap feed = maps.feed(left, overNet);
        assertNotNull(feed, "left=" + left + " overNet=" + overNet);
        assertEquals("feed", feed.kind());
        assertTrue(feed.target().isPresent());
      }
    }
  }

  @Test
  void missingFilesAreReportedNotThrown(@TempDir Path empty) {
    ShotMaps maps = new ShotMaps(empty);
    assertFalse(maps.available());
    assertNull(maps.hub());
    assertNull(maps.calibration());
    assertTrue(maps.loadErrors().contains("shotmaps/hub.json"));
    assertTrue(maps.loadErrors().contains("shooter_calibration.json"));
  }
}
