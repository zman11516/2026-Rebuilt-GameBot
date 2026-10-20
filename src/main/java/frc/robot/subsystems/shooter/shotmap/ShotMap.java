package frc.robot.subsystems.shooter.shotmap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

/**
 * A shot map generated offline by tools/shotsim (schema "shotsim5454/1"): the simulated shot for
 * every (distance, radial velocity) grid point, plus polynomials fitted to them.
 *
 * <p>Port of tools/shotsim/lib/mapeval.mjs. The parity tests compare it against values that file
 * produced, so keep the two in step.
 */
public final class ShotMap {
  public enum Mode {
    /** Fitted polynomials. */
    POLY,
    /** Bilinear interpolation of the raw simulated grid. */
    GRID
  }

  public static final String SCHEMA = "shotsim5454/1";

  private static final ObjectMapper MAPPER = new ObjectMapper();
  private static final String[] OUTPUTS = {
    "angleDeg",
    "speed",
    "tof",
    "angleMarginLow",
    "angleMarginHigh",
    "speedMarginLow",
    "speedMarginHigh"
  };

  private final String name;
  private final String kind;
  private final Mode recommendedMode;
  private final Optional<Translation2d> target;

  private final double[] gridD;
  private final double[] gridVr;
  private final double dMid, dHalf, rMid, rHalf;
  private final int[][] basis;
  private final double[][] coefficients; // [output][basis term]
  // [output][vr index][d index]; NaN where the export has null (no valid shot)
  private final double[][][] rawGrid;
  private final double[][] vtMaxGrid;

  private final double boundsDMin, boundsDMax, boundsRMin, boundsRMax;
  // Valid distances per grid row; NaN when the row has none
  private final double[] rowVr, rowDMin, rowDMax;
  private final double vtGlobal;
  // Feed maps: obstacle footprints {x0, x1, y0, y1} (blue frame) that pick feed vs net feed
  private final double[][] crossingBoxes;

  /** Read a map exported by tools/shotsim. */
  public static ShotMap load(Path file) throws IOException {
    return fromJson(MAPPER.readTree(file.toFile()));
  }

  /** Build a map from its parsed JSON; throws IllegalArgumentException if a field is missing. */
  public static ShotMap fromJson(JsonNode node) {
    return new ShotMap(node);
  }

  private ShotMap(JsonNode node) {
    String schema = node.path("schema").asText();
    if (!SCHEMA.equals(schema)) {
      throw new IllegalArgumentException("unsupported schema '" + schema + "'");
    }
    name = node.path("name").asText();
    kind = node.path("kind").asText();
    recommendedMode = "GRID".equals(node.path("recommendedMode").asText()) ? Mode.GRID : Mode.POLY;
    JsonNode t = node.path("target");
    target =
        t.isArray() && t.size() == 2
            ? Optional.of(new Translation2d(t.get(0).asDouble(), t.get(1).asDouble()))
            : Optional.empty();

    gridD = doubles(node.path("grid").path("d"), "grid.d", false);
    gridVr = doubles(node.path("grid").path("vr"), "grid.vr", false);
    if (gridD.length < 2 || gridVr.length < 2) {
      throw new IllegalArgumentException("grid needs at least 2 points per axis");
    }
    dMid = requireNumber(node, "normalization.dMid");
    dHalf = requireNumber(node, "normalization.dHalf");
    rMid = requireNumber(node, "normalization.rMid");
    rHalf = requireNumber(node, "normalization.rHalf");

    JsonNode basisNode = node.path("basis");
    if (basisNode.size() == 0) throw new IllegalArgumentException("missing basis");
    basis = new int[basisNode.size()][];
    for (int k = 0; k < basis.length; k++) {
      basis[k] = new int[] {basisNode.get(k).get(0).asInt(), basisNode.get(k).get(1).asInt()};
    }
    coefficients = new double[OUTPUTS.length][];
    rawGrid = new double[OUTPUTS.length][][];
    for (int i = 0; i < OUTPUTS.length; i++) {
      coefficients[i] =
          doubles(node.path("coefficients").path(OUTPUTS[i]), "coefficients." + OUTPUTS[i], false);
      if (coefficients[i].length != basis.length) {
        throw new IllegalArgumentException("coefficients." + OUTPUTS[i] + " does not match basis");
      }
      rawGrid[i] = grid2d(node.path("rawGrid").path(OUTPUTS[i]), "rawGrid." + OUTPUTS[i]);
    }
    vtMaxGrid = grid2d(node.path("rawGrid").path("vtMax"), "rawGrid.vtMax");

    boundsDMin = requireNumber(node, "bounds.dMin");
    boundsDMax = requireNumber(node, "bounds.dMax");
    boundsRMin = requireNumber(node, "bounds.rMin");
    boundsRMax = requireNumber(node, "bounds.rMax");
    JsonNode rows = node.path("rows");
    if (rows.size() == 0) throw new IllegalArgumentException("missing rows");
    rowVr = new double[rows.size()];
    rowDMin = new double[rows.size()];
    rowDMax = new double[rows.size()];
    for (int i = 0; i < rows.size(); i++) {
      JsonNode row = rows.get(i);
      rowVr[i] = requireNumber(row, "vr");
      rowDMin[i] = row.path("dMin").isNumber() ? row.get("dMin").asDouble() : Double.NaN;
      rowDMax[i] = row.path("dMax").isNumber() ? row.get("dMax").asDouble() : Double.NaN;
    }
    vtGlobal = requireNumber(node, "tangentialLimit.global");

    JsonNode boxes = node.path("crossing").path("boxes");
    crossingBoxes = new double[boxes.size()][];
    for (int i = 0; i < boxes.size(); i++) {
      double[] x = doubles(boxes.get(i).path("x"), "crossing.boxes.x", false);
      double[] y = doubles(boxes.get(i).path("y"), "crossing.boxes.y", false);
      if (x.length != 2 || y.length != 2) throw new IllegalArgumentException("bad crossing box");
      crossingBoxes[i] = new double[] {x[0], x[1], y[0], y[1]};
    }
  }

  public String name() {
    return name;
  }

  /** "hub" or "feed". */
  public String kind() {
    return kind;
  }

  /** POLY when the fit tracks the simulated shots closely enough, otherwise GRID. */
  public Mode recommendedMode() {
    return recommendedMode;
  }

  /** Feed maps: the target in blue-alliance field coordinates. Empty for the hub. */
  public Optional<Translation2d> target() {
    return target;
  }

  /**
   * Evaluate the map at a distance (m) and radial velocity (m/s, + closing on the target). Values
   * are filled from the clamped inputs even when the result is not valid.
   */
  public ShotMapOutput eval(double distance, double radialVelocity, Mode mode) {
    boolean valid = true;
    String reason = "OK";
    double[] range = distanceRange(radialVelocity);
    if (range == null) {
      valid = false;
      reason = "RADIAL_OUT_OF_RANGE";
    } else if (distance < range[0] || distance > range[1]) {
      valid = false;
      reason = "DISTANCE_OUT_OF_RANGE";
    }
    double dc =
        Math.min(
            Math.max(distance, range != null ? range[0] : boundsDMin),
            range != null ? range[1] : boundsDMax);
    double rc = Math.min(Math.max(radialVelocity, boundsRMin), boundsRMax);

    double[] out = new double[OUTPUTS.length];
    for (int i = 0; i < OUTPUTS.length; i++) {
      Double v =
          mode == Mode.GRID ? gridValue(rawGrid[i], dc, rc) : Double.valueOf(polyValue(i, dc, rc));
      if (v == null) {
        // A corner of this grid cell was not a valid shot: keep the turret tracking with the fit
        v = polyValue(i, dc, rc);
        if (valid) {
          valid = false;
          reason = "GRID_CELL_INVALID";
        }
      }
      out[i] = OUTPUTS[i].contains("Margin") ? Math.max(0, v) : v;
    }
    Double vt = gridValue(vtMaxGrid, dc, rc);
    return new ShotMapOutput(
        valid,
        reason,
        out[0],
        out[1],
        out[2],
        out[3],
        out[4],
        out[5],
        out[6],
        vt == null ? vtGlobal : vt);
  }

  /** Feed maps: true when the segment crosses any crossing box (blue frame). */
  public boolean lineCrossesBoxes(Translation2d fromBlue, Translation2d toBlue) {
    for (double[] b : crossingBoxes) {
      if (segmentHitsBox(fromBlue.getX(), fromBlue.getY(), toBlue.getX(), toBlue.getY(), b)) {
        return true;
      }
    }
    return false;
  }

  /** Value of polynomial output i at (d, vr). */
  private double polyValue(int i, double d, double vr) {
    double dn = (d - dMid) / dHalf;
    double rn = (vr - rMid) / rHalf;
    double[] c = coefficients[i];
    double v = 0;
    for (int k = 0; k < basis.length; k++) {
      v += c[k] * Math.pow(dn, basis[k][0]) * Math.pow(rn, basis[k][1]);
    }
    return v;
  }

  private record Bracket(int index, double fraction) {}

  /** Index i with axis[i] <= v <= axis[i+1] (clamped) and the fraction between them. */
  private static Bracket locate(double[] axis, double v) {
    if (v <= axis[0]) return new Bracket(0, 0);
    int last = axis.length - 1;
    if (v >= axis[last]) return new Bracket(Math.max(0, last - 1), last == 0 ? 0 : 1);
    int i = 0;
    while (i < last - 1 && axis[i + 1] < v) i++;
    return new Bracket(i, (v - axis[i]) / (axis[i + 1] - axis[i]));
  }

  /** Bilinear value of a raw grid; null when any of the four corners is invalid. */
  private Double gridValue(double[][] g, double d, double vr) {
    Bracket bd = locate(gridD, d);
    Bracket br = locate(gridVr, vr);
    int di = bd.index();
    int ri = br.index();
    int di1 = Math.min(di + 1, gridD.length - 1);
    int ri1 = Math.min(ri + 1, gridVr.length - 1);
    double v00 = g[ri][di];
    double v01 = g[ri][di1];
    double v10 = g[ri1][di];
    double v11 = g[ri1][di1];
    if (Double.isNaN(v00) || Double.isNaN(v01) || Double.isNaN(v10) || Double.isNaN(v11)) {
      return null;
    }
    double fd = bd.fraction();
    double fr = br.fraction();
    return (1 - fr) * ((1 - fd) * v00 + fd * v01) + fr * ((1 - fd) * v10 + fd * v11);
  }

  /** Valid {dMin, dMax} for a radial velocity: the intersection of the two bracketing rows. */
  private double[] distanceRange(double vr) {
    if (vr < boundsRMin || vr > boundsRMax) return null;
    int i = 0;
    while (i < rowVr.length - 2 && rowVr[i + 1] < vr) i++;
    int j = Math.min(i + 1, rowVr.length - 1);
    if (Double.isNaN(rowDMin[i]) || Double.isNaN(rowDMin[j])) return null;
    double dMin = Math.max(rowDMin[i], rowDMin[j]);
    double dMax = Math.min(rowDMax[i], rowDMax[j]);
    return dMin <= dMax ? new double[] {dMin, dMax} : null;
  }

  /** Liang-Barsky clip of the segment A->B against box {x0, x1, y0, y1} (targets.mjs segRect). */
  private static boolean segmentHitsBox(double ax, double ay, double bx, double by, double[] box) {
    double dx = bx - ax;
    double dy = by - ay;
    double[] p = {-dx, dx, -dy, dy};
    double[] q = {ax - box[0], box[1] - ax, ay - box[2], box[3] - ay};
    double t0 = 0;
    double t1 = 1;
    for (int k = 0; k < 4; k++) {
      if (Math.abs(p[k]) < 1e-12) {
        if (!(q[k] >= 0)) return false;
        continue;
      }
      double r = q[k] / p[k];
      if (p[k] < 0) {
        if (r > t1) return false;
        if (r > t0) t0 = r;
      } else {
        if (r < t0) return false;
        if (r < t1) t1 = r;
      }
    }
    return true;
  }

  /** Number at a dotted path such as "hood.theta0Deg"; throws if it is missing. */
  static double requireNumber(JsonNode root, String path) {
    JsonNode n = root;
    for (String key : path.split("\\.")) n = n.path(key);
    if (!n.isNumber()) throw new IllegalArgumentException("missing number '" + path + "'");
    return n.asDouble();
  }

  private static double[] doubles(JsonNode array, String what, boolean allowNull) {
    if (!array.isArray()) throw new IllegalArgumentException("missing array '" + what + "'");
    double[] out = new double[array.size()];
    for (int i = 0; i < out.length; i++) {
      JsonNode v = array.get(i);
      if (v.isNumber()) {
        out[i] = v.asDouble();
      } else if (allowNull && v.isNull()) {
        out[i] = Double.NaN;
      } else {
        throw new IllegalArgumentException("bad value in '" + what + "'");
      }
    }
    return out;
  }

  private double[][] grid2d(JsonNode rows, String what) {
    if (rows.size() != gridVr.length) throw new IllegalArgumentException("bad shape '" + what + "'");
    double[][] out = new double[rows.size()][];
    for (int i = 0; i < out.length; i++) {
      out[i] = doubles(rows.get(i), what, true);
      if (out[i].length != gridD.length) {
        throw new IllegalArgumentException("bad shape '" + what + "'");
      }
    }
    return out;
  }
}
