// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Everything {@link frc.robot.subsystems.Vision} needs, with no reference to a specific drivetrain.
 * Headings and poses arrive as suppliers so this whole package can move into the robot library
 * unchanged.
 *
 * @param headingDegrees field-relative, blue-origin robot heading for MegaTag2 (0 deg = facing the
 *     red wall). Use the pose estimator's rotation, not the raw gyro: the estimator carries the
 *     reset offset, the raw gyro only matches if it happened to be zeroed at the red wall.
 * @param yawRateDegPerSec angular velocity about world Z; pass {@code () -> 0.0} if the sign
 *     convention is in doubt, the Limelight documents this input as optional
 * @param currentPose current best estimate, used only for the teleport/outlier gate
 * @param fuseAllCameras when true every accepted camera is reported; when false only the
 *     highest-scoring one is
 */
public record VisionConfig(
    List<CameraConfig> cameras,
    DoubleSupplier headingDegrees,
    DoubleSupplier yawRateDegPerSec,
    Supplier<Pose2d> currentPose,
    FieldBounds bounds,
    AcceptanceParams acceptance,
    StdDevParams stdDevs,
    boolean fuseAllCameras,
    boolean logTelemetry) {

  /** Sensible defaults: standard acceptance and std devs, single best camera, telemetry on. */
  public VisionConfig(
      List<CameraConfig> cameras,
      DoubleSupplier headingDegrees,
      DoubleSupplier yawRateDegPerSec,
      Supplier<Pose2d> currentPose,
      FieldBounds bounds) {
    this(
        cameras,
        headingDegrees,
        yawRateDegPerSec,
        currentPose,
        bounds,
        AcceptanceParams.defaults(),
        StdDevParams.defaults(),
        false,
        true);
  }

  /**
   * One Limelight.
   *
   * @param name its NetworkTables table name, e.g. "limelight-left"
   * @param trust relative weight; 1.0 is nominal, below 1.0 de-weights a camera whose mounting is
   *     less certain (a turret-mounted camera is only as good as the turret encoder)
   * @param enabled false skips the camera entirely without removing it from the list
   */
  public record CameraConfig(String name, double trust, boolean enabled) {

    public CameraConfig(String name) {
      this(name, 1.0, true);
    }

    public CameraConfig(String name, double trust) {
      this(name, trust, true);
    }
  }

  /**
   * Field extents used as a sanity gate on vision poses. Tolerance allows a small amount of
   * overhang past the field edge (a real pose can sit slightly outside when the robot is against a
   * wall) without accepting obvious garbage.
   */
  public record FieldBounds(double xMin, double yMin, double xMax, double yMax, double tolerance) {

    /** Bounds anchored at the origin with the default 0.4 m tolerance. */
    public FieldBounds(double xMax, double yMax) {
      this(0.0, 0.0, xMax, yMax, 0.4);
    }

    public boolean contains(double x, double y) {
      return x >= xMin - tolerance
          && x <= xMax + tolerance
          && y >= yMin - tolerance
          && y <= yMax + tolerance;
    }
  }

  /**
   * Thresholds deciding whether a {@link CameraSample} is trustworthy enough to hand to the pose
   * estimator. Single-tag solutions get much tighter gates than multi-tag ones: a single tag has no
   * baseline, so its solve is poorly conditioned and its pose can flip.
   *
   * @param maxSingleTagDist max average tag distance, meters, when only one tag is visible
   * @param maxMultiTagDist max average tag distance, meters, with two or more tags
   * @param maxSingleTagAmbiguity max per-tag ambiguity tolerated for a single-tag solution
   * @param minSingleTagArea min average tag area (fraction of image) for a single-tag solution
   * @param maxJumpSingleTag max distance, meters, a single-tag pose may sit from the current estimate
   * @param maxJumpMultiTag max distance, meters, a multi-tag pose may sit from the current estimate
   */
  public record AcceptanceParams(
      double maxSingleTagDist,
      double maxMultiTagDist,
      double maxSingleTagAmbiguity,
      double minSingleTagArea,
      double maxJumpSingleTag,
      double maxJumpMultiTag) {

    public static AcceptanceParams defaults() {
      return new AcceptanceParams(4.0, 6.5, 0.3, 0.08, 1.0, 2.5);
    }
  }

  /**
   * Coefficients for the vision measurement standard deviations handed to the pose estimator.
   *
   * @param baseXy scales the distance^2 / tagCount trust curve, meters
   * @param minXy floor on the xy standard deviation, meters
   * @param maxXy ceiling on the xy standard deviation, meters
   * @param theta heading standard deviation, radians -- see {@link #defaults()}
   */
  public record StdDevParams(double baseXy, double minXy, double maxXy, double theta) {

    /**
     * MegaTag2's reported rotation is the heading we pushed to the Limelight, so trusting it back
     * would be circular -- theta is pinned at a huge value to make the estimator ignore it.
     *
     * <p>Deliberately 9_999_999.0 and not {@link Double#POSITIVE_INFINITY}: the estimator squares
     * these into a measurement covariance and solves for a Kalman gain, and infinity risks
     * propagating NaN through that computation. The magic number is the standard FRC idiom here.
     */
    public static StdDevParams defaults() {
      return new StdDevParams(0.06, 0.05, 4.0, 9_999_999.0);
    }
  }
}
