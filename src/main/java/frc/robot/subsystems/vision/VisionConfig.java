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
}
