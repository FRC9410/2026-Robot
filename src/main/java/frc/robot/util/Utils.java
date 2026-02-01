package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ReefConstants;
import frc.robot.Constants.WaypointConstants;
import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Utils {
  /**
   * Gets the current alliance color (Red or Blue)
   *
   * @return Alliance enum representing the current alliance color. Returns null if alliance color
   *     is not yet available.
   */
  public static String getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red ? "red" : "blue";
    }
    return "";
  }

  /**
   * Checks if a value is within a specified tolerance of a target
   *
   * @param current The current value
   * @param target The target value
   * @param tolerance The acceptable tolerance (must be positive)
   * @return true if the current value is within the tolerance of the target
   */
  public static boolean isWithinTolerance(double current, double target, double tolerance) {
    return Math.abs(current - target) <= Math.abs(tolerance);
  }

  /**
   * Applies a curve to driver input to allow for finer control at low speeds while still allowing
   * for full speed. Preserves the sign of the input.
   *
   * @param input The raw driver input (should be between -1.0 and 1.0)
   * @param exponent The power to raise the input to (2 for square, 3 for cube, etc.)
   * @return The curved value, maintaining the sign of the input
   */
  public static double curveInput(double input, double exponent) {
    double sign = Math.signum(input);
    return sign * Math.pow(Math.abs(input), exponent);
  }

  /**
   * Applies a squared curve to driver input to allow for finer control at low speeds while still
   * allowing for full speed. Preserves the sign of the input.
   *
   * @param input The raw driver input (should be between -1.0 and 1.0)
   * @return The curved value, maintaining the sign of the input
   */
  public static double squareInput(double input) {
    if (input < 0) {
      return -curveInput(-input, 2.0);
    }
    return curveInput(input, 2.0);
  }

  /**
   * Applies a cubed curve to driver input to allow for finer control at low speeds while still
   * allowing for full speed. Preserves the sign of the input.
   *
   * @param input The raw driver input (should be between -1.0 and 1.0)
   * @return The curved value, maintaining the sign of the input
   */
  public static double cubeInput(double input) {
    return curveInput(input, 3.0);
  }

  public static void logMap(Map<String, Object> map, String tableName) {
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final NetworkTable table = inst.getTable(tableName);
    for (Map.Entry<String, Object> entry : map.entrySet()) {
      if (entry.getValue() == null || entry.getKey() == null) {
        System.out.println(entry.getKey());
        continue;
      }
      if (entry.getValue() instanceof Double) {
        table.getEntry(entry.getKey()).setDouble((Double) entry.getValue());
      } else if (entry.getValue() instanceof Integer) {
        table.getEntry(entry.getKey()).setNumber((Integer) entry.getValue());
      } else if (entry.getValue() instanceof Boolean) {
        table.getEntry(entry.getKey()).setBoolean((Boolean) entry.getValue());
      } else {
        table.getEntry(entry.getKey()).setString(entry.getValue().toString());
      }
    }
  }

  // Define field center (adjust as needed)
  private static final Point2D.Double RED_FIELD_CENTER = new Point2D.Double(13.066, 4.02);
  private static final Point2D.Double BLUE_FIELD_CENTER = new Point2D.Double(4.49, 4.02);

  private static final Map<String, Double> BLUE_ZONE_ANGLES = new LinkedHashMap<>();

  static {
    BLUE_ZONE_ANGLES.put("ZONE_1", WaypointConstants.BLUE_FRONT_LEFT.getRotation().getDegrees());
    BLUE_ZONE_ANGLES.put("ZONE_2", WaypointConstants.BLUE_LEFT.getRotation().getDegrees());
    BLUE_ZONE_ANGLES.put("ZONE_3", WaypointConstants.BLUE_BACK_LEFT.getRotation().getDegrees());
    BLUE_ZONE_ANGLES.put("ZONE_4", WaypointConstants.BLUE_BACK_RIGHT.getRotation().getDegrees());
    BLUE_ZONE_ANGLES.put("ZONE_5", WaypointConstants.BLUE_RIGHT.getRotation().getDegrees());
    BLUE_ZONE_ANGLES.put("ZONE_6", WaypointConstants.BLUE_FRONT_RIGHT.getRotation().getDegrees());
  }

  private static final Map<String, Double> RED_ZONE_ANGLES = new LinkedHashMap<>();

  static {
    RED_ZONE_ANGLES.put("ZONE_1", WaypointConstants.RED_FRONT_LEFT.getRotation().getDegrees());
    RED_ZONE_ANGLES.put("ZONE_2", WaypointConstants.RED_LEFT.getRotation().getDegrees());
    RED_ZONE_ANGLES.put("ZONE_3", WaypointConstants.RED_BACK_LEFT.getRotation().getDegrees());
    RED_ZONE_ANGLES.put("ZONE_4", WaypointConstants.RED_BACK_RIGHT.getRotation().getDegrees());
    RED_ZONE_ANGLES.put("ZONE_5", WaypointConstants.RED_RIGHT.getRotation().getDegrees());
    RED_ZONE_ANGLES.put("ZONE_6", WaypointConstants.RED_FRONT_RIGHT.getRotation().getDegrees());
  }
  /** Determines which zone a given position belongs to. */
  public static String getZone(Point2D.Double position) {
    Point2D.Double FIELD_CENTER =
        getAllianceColor().equals("red") ? RED_FIELD_CENTER : BLUE_FIELD_CENTER;
    double angle = computeAngle(FIELD_CENTER, position);
    Map<String, Double> ZONE_ANGLES =
        getAllianceColor().equals("red") ? RED_ZONE_ANGLES : BLUE_ZONE_ANGLES;

    // Find the appropriate zone based on angle
    String lastZone = null;
    for (Map.Entry<String, Double> entry : ZONE_ANGLES.entrySet()) {
      if (angle < entry.getValue()) {
        return lastZone != null ? lastZone : entry.getKey();
      }
      lastZone = entry.getKey();
    }
    return "ZONE_1"; // Wrap around (default to first zone if angle exceeds last boundary)
  }

  /** Computes the angle (in degrees) from the center to a given point. */
  private static double computeAngle(Point2D.Double center, Point2D.Double point) {
    double dx = point.x - center.x;
    double dy = point.y - center.y;
    double radians = Math.atan2(dy, dx);
    double degrees = Math.toDegrees(radians);

    return (degrees + 360) % 360; // Normalize angle to range [0, 360)
  }

  // Define hexagon vertices (assuming known, in counterclockwise order)
  private static final List<Point2D.Double> BLUE_HEXAGON_VERTICES =
      Arrays.asList(
          ReefConstants.BLUE_BACK_LEFT,
          ReefConstants.BLUE_FRONT_LEFT,
          ReefConstants.BLUE_FRONT_RIGHT,
          ReefConstants.BLUE_BACK_RIGHT,
          ReefConstants.BLUE_LEFT,
          ReefConstants.BLUE_RIGHT);

  private static final List<Point2D.Double> RED_HEXAGON_VERTICES =
      Arrays.asList(
          ReefConstants.RED_BACK_LEFT,
          ReefConstants.RED_FRONT_LEFT,
          ReefConstants.RED_FRONT_RIGHT,
          ReefConstants.RED_BACK_RIGHT,
          ReefConstants.RED_LEFT,
          ReefConstants.RED_RIGHT);

  // Define predefined safe waypoints around the hexagon
  private static final List<Point2D.Double> BLUE_SAFE_WAYPOINTS =
      Arrays.asList(
          new Point2D.Double(
              WaypointConstants.BLUE_FRONT_LEFT.getX(), WaypointConstants.BLUE_FRONT_LEFT.getY()),
          new Point2D.Double(
              WaypointConstants.BLUE_LEFT.getX(), WaypointConstants.BLUE_LEFT.getY()),
          new Point2D.Double(
              WaypointConstants.BLUE_BACK_LEFT.getX(), WaypointConstants.BLUE_BACK_LEFT.getY()),
          new Point2D.Double(
              WaypointConstants.BLUE_BACK_RIGHT.getX(), WaypointConstants.BLUE_BACK_RIGHT.getY()),
          new Point2D.Double(
              WaypointConstants.BLUE_RIGHT.getX(), WaypointConstants.BLUE_RIGHT.getY()),
          new Point2D.Double(
              WaypointConstants.BLUE_FRONT_RIGHT.getX(),
              WaypointConstants.BLUE_FRONT_RIGHT.getY()));

  private static final List<Point2D.Double> RED_SAFE_WAYPOINTS =
      Arrays.asList(
          new Point2D.Double(
              WaypointConstants.RED_FRONT_LEFT.getX(), WaypointConstants.RED_FRONT_LEFT.getY()),
          new Point2D.Double(WaypointConstants.RED_LEFT.getX(), WaypointConstants.RED_LEFT.getY()),
          new Point2D.Double(
              WaypointConstants.RED_BACK_LEFT.getX(), WaypointConstants.RED_BACK_LEFT.getY()),
          new Point2D.Double(
              WaypointConstants.RED_BACK_RIGHT.getX(), WaypointConstants.RED_BACK_RIGHT.getY()),
          new Point2D.Double(
              WaypointConstants.RED_RIGHT.getX(), WaypointConstants.RED_RIGHT.getY()),
          new Point2D.Double(
              WaypointConstants.RED_FRONT_RIGHT.getX(), WaypointConstants.RED_FRONT_RIGHT.getY()));

  // Ordered list of zones (Clockwise order)
  private static final List<String> ZONES =
      Arrays.asList("ZONE_1", "ZONE_2", "ZONE_3", "ZONE_4", "ZONE_5", "ZONE_6");

  // Lookup table for waypoints when transitioning between zones
  private static final Map<String, Point2D.Double> RED_WAYPOINT_LOOKUP = new HashMap<>();

  static {
    RED_WAYPOINT_LOOKUP.put("ZONE_1->ZONE_2", RED_SAFE_WAYPOINTS.get(1));
    RED_WAYPOINT_LOOKUP.put("ZONE_2->ZONE_3", RED_SAFE_WAYPOINTS.get(0));
    RED_WAYPOINT_LOOKUP.put("ZONE_3->ZONE_4", RED_SAFE_WAYPOINTS.get(5));
    RED_WAYPOINT_LOOKUP.put("ZONE_4->ZONE_5", RED_SAFE_WAYPOINTS.get(4));
    RED_WAYPOINT_LOOKUP.put("ZONE_5->ZONE_6", RED_SAFE_WAYPOINTS.get(3));
    RED_WAYPOINT_LOOKUP.put("ZONE_6->ZONE_1", RED_SAFE_WAYPOINTS.get(2));
    RED_WAYPOINT_LOOKUP.put("ZONE_1->ZONE_6", RED_SAFE_WAYPOINTS.get(2));
    RED_WAYPOINT_LOOKUP.put("ZONE_6->ZONE_5", RED_SAFE_WAYPOINTS.get(3));
    RED_WAYPOINT_LOOKUP.put("ZONE_5->ZONE_4", RED_SAFE_WAYPOINTS.get(4));
    RED_WAYPOINT_LOOKUP.put("ZONE_4->ZONE_3", RED_SAFE_WAYPOINTS.get(5));
    RED_WAYPOINT_LOOKUP.put("ZONE_3->ZONE_2", RED_SAFE_WAYPOINTS.get(0));
    RED_WAYPOINT_LOOKUP.put("ZONE_2->ZONE_1", RED_SAFE_WAYPOINTS.get(2));
  }

  private static final Map<String, Point2D.Double> BLUE_WAYPOINT_LOOKUP = new HashMap<>();

  static {
    BLUE_WAYPOINT_LOOKUP.put("ZONE_1->ZONE_2", BLUE_SAFE_WAYPOINTS.get(0));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_2->ZONE_3", BLUE_SAFE_WAYPOINTS.get(1));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_3->ZONE_4", BLUE_SAFE_WAYPOINTS.get(2));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_4->ZONE_5", BLUE_SAFE_WAYPOINTS.get(3));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_5->ZONE_6", BLUE_SAFE_WAYPOINTS.get(4));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_6->ZONE_1", BLUE_SAFE_WAYPOINTS.get(5));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_1->ZONE_6", BLUE_SAFE_WAYPOINTS.get(5));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_6->ZONE_5", BLUE_SAFE_WAYPOINTS.get(4));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_5->ZONE_4", BLUE_SAFE_WAYPOINTS.get(3));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_4->ZONE_3", BLUE_SAFE_WAYPOINTS.get(2));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_3->ZONE_2", BLUE_SAFE_WAYPOINTS.get(1));
    BLUE_WAYPOINT_LOOKUP.put("ZONE_2->ZONE_1", BLUE_SAFE_WAYPOINTS.get(0));
  }

  /** Finds the index of a zone in the ordered zone list. */
  private static int getZoneIndex(String zone) {
    return ZONES.indexOf(zone);
  }

  /** Finds the next zone to travel to based on the shortest route. */
  private static String getNextZone(String startZone, String targetZone) {
    int startIdx = getZoneIndex(startZone);
    int targetIdx = getZoneIndex(targetZone);

    if (startIdx == -1 || targetIdx == -1 || startIdx == targetIdx) {
      return null; // Invalid zones or already at target
    }

    // Calculate clockwise and counterclockwise distances
    int clockwiseSteps = (targetIdx - startIdx + ZONES.size()) % ZONES.size();
    int counterClockwiseSteps = (startIdx - targetIdx + ZONES.size()) % ZONES.size();

    // Choose the shortest direction and return the next zone
    if (clockwiseSteps <= counterClockwiseSteps) {
      return ZONES.get((startIdx + 1) % ZONES.size());
    } else {
      return ZONES.get((startIdx - 1 + ZONES.size()) % ZONES.size());
    }
  }

  /** Gets the next waypoint based on the shortest zone transition. */
  public static Point2D.Double getNextWaypoint(String startZone, String targetZone) {
    Map<String, Point2D.Double> WAYPOINT_LOOKUP =
        getAllianceColor().equals("red") ? RED_WAYPOINT_LOOKUP : BLUE_WAYPOINT_LOOKUP;
    String nextZone = getNextZone(startZone, targetZone);
    System.out.println("Start Zone: " + startZone);
    System.out.println("Next Zone: " + nextZone);
    if (nextZone == null) {
      return null; // No waypoint needed (already in the target zone)
    }

    String key = startZone + "->" + nextZone;
    return WAYPOINT_LOOKUP.getOrDefault(key, null);
  }

  public static Pose2d getNextPose(Pose2d currentPose, Pose2d targetPose) {
    Point2D.Double currentPoint = new Point2D.Double(currentPose.getX(), currentPose.getY());
    Point2D.Double targetPoint = new Point2D.Double(targetPose.getX(), targetPose.getY());
    String startZone = getZone(currentPoint);
    String targetZone = getZone(targetPoint);

    if (startZone.equals(targetZone)) {
      return targetPose; // Already in the target zone
    }

    Point2D.Double nextWaypoint = getNextWaypoint(startZone, targetZone);
    if (nextWaypoint == null) {
      return targetPose; // No waypoint needed (already in the target zone)
    }

    return new Pose2d(nextWaypoint.getX(), nextWaypoint.getY(), targetPose.getRotation());
  }
}
