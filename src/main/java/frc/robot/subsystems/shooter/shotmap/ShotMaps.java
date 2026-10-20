package frc.robot.subsystems.shooter.shotmap;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

/**
 * The shot maps and shooter calibration from the deploy directory, loaded once. Never throws: a
 * file that fails to load is null and its error is listed in {@link #loadErrors()}.
 */
public final class ShotMaps {
  private static ShotMaps instance;

  private final ShotMap hub;
  private final ShotMap feedLeft;
  private final ShotMap feedRight;
  private final ShotMap netFeedLeft;
  private final ShotMap netFeedRight;
  private final ShooterCalibration calibration;
  private final String loadErrors;

  public static synchronized ShotMaps get() {
    if (instance == null) instance = new ShotMaps(Filesystem.getDeployDirectory().toPath());
    return instance;
  }

  ShotMaps(Path deployDir) {
    StringBuilder errors = new StringBuilder();
    hub = tryLoad(deployDir, "shotmaps/hub.json", ShotMap::load, errors);
    feedLeft = tryLoad(deployDir, "shotmaps/feed_left.json", ShotMap::load, errors);
    feedRight = tryLoad(deployDir, "shotmaps/feed_right.json", ShotMap::load, errors);
    netFeedLeft = tryLoad(deployDir, "shotmaps/netfeed_left.json", ShotMap::load, errors);
    netFeedRight = tryLoad(deployDir, "shotmaps/netfeed_right.json", ShotMap::load, errors);
    calibration =
        tryLoad(deployDir, "shooter_calibration.json", ShooterCalibration::load, errors);
    loadErrors = errors.toString().strip();
  }

  /** True when the hub map and the calibration loaded. */
  public boolean available() {
    return hub != null && calibration != null;
  }

  /** Hub map, or null if it failed to load. */
  public ShotMap hub() {
    return hub;
  }

  /** Feed map for the left or right target, over the net or not; null if it failed to load. */
  public ShotMap feed(boolean left, boolean overNet) {
    if (left) return overNet ? netFeedLeft : feedLeft;
    return overNet ? netFeedRight : feedRight;
  }

  /** Calibration, or null if it failed to load. */
  public ShooterCalibration calibration() {
    return calibration;
  }

  /** One line per file that failed to load; empty when everything loaded. */
  public String loadErrors() {
    return loadErrors;
  }

  private interface Loader<T> {
    T load(Path file) throws IOException;
  }

  private static <T> T tryLoad(
      Path deployDir, String file, Loader<T> loader, StringBuilder errors) {
    try {
      return loader.load(deployDir.resolve(file));
    } catch (Exception e) {
      errors.append(file).append(": ").append(e).append('\n');
      return null;
    }
  }
}
