package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Vision;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class VisionConstants {
  public static final String LEFT_TABLE = "limelight-left";
  public static final String RIGHT_TABLE = "limelight-right";
  public static final String TURRET_TABLE = "limelight-turret";

  /**
   * Relative weight per camera. The turret camera sits on a rotating mechanism, so its robot-space
   * transform is only as good as the turret encoder -- it starts trusted a little less than the
   * fixed side cameras.
   */
  public static final double LEFT_TRUST = 1.0;

  public static final double RIGHT_TRUST = 1.0;
  public static final double TURRET_TRUST = 0.8;

  /**
   * Off until bench validation. Several cameras looking at the same tags produce correlated errors,
   * which makes the estimator more confident than the evidence warrants; with this false only the
   * single best camera contributes each loop.
   */
  public static final boolean FUSE_ALL_CAMERAS = false;

  /**
   * How far past the field edge a pose may sit and still be believed. A real pose can overhang
   * slightly when the robot is against a wall; anything further is garbage.
   */
  public static final double FIELD_TOLERANCE_METERS = 0.4;

  /** Builds the vision configuration. Suppliers come from the drivetrain. */
  public static Vision.Config config(
      DoubleSupplier headingDegrees, DoubleSupplier yawRateDegPerSec, Supplier<Pose2d> currentPose) {
    return new Vision.Config(
        List.of(
            new Vision.CameraConfig(LEFT_TABLE, LEFT_TRUST),
            new Vision.CameraConfig(RIGHT_TABLE, RIGHT_TRUST),
            new Vision.CameraConfig(TURRET_TABLE, TURRET_TRUST)),
        headingDegrees,
        yawRateDegPerSec,
        currentPose,
        new Vision.FieldBounds(
            FieldConstants.X_MIN,
            FieldConstants.Y_MIN,
            FieldConstants.X_MAX,
            FieldConstants.Y_MAX,
            FIELD_TOLERANCE_METERS),
        Vision.AcceptanceParams.defaults(),
        Vision.StdDevParams.defaults(),
        FUSE_ALL_CAMERAS,
        true);
  }
}
