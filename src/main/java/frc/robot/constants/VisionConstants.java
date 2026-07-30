package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Vision;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// Tuning for the vision system. Everything here is meant to be edited from bench data -- the
// starting values are conventional FRC defaults, not measurements taken on this robot.
public class VisionConstants {

  // NetworkTables table names. These must match what each Limelight is configured as in its own web
  // UI -- they name the device, not what it is bolted to. A mismatch reads as an absent camera.
  public static final String LEFT_TABLE = "limelight-left";
  public static final String RIGHT_TABLE = "limelight-right";
  public static final String TURRET_TABLE = "limelight-turret";

  // Relative weight per camera. All three are rigidly mounted to the chassis, so none has a less
  // certain robot-space transform than the others and they all start equal. Drop one below 1.0 if
  // bench data shows it disagreeing with the others.
  public static final double LEFT_TRUST = 1.0;
  public static final double RIGHT_TRUST = 1.0;
  public static final double TURRET_TRUST = 1.0;

  // Off until bench validation. Several cameras looking at the same tags produce correlated errors,
  // which makes the estimator more confident than the evidence warrants. With this false only the
  // single best camera contributes each loop.
  public static final boolean FUSE_ALL_CAMERAS = false;

  // How far past the field edge a pose may sit and still be believed. A real pose can overhang
  // slightly when the robot is pressed against a wall; anything further is garbage.
  public static final double FIELD_TOLERANCE_METERS = 0.4;

  // Absolute pitch or roll beyond which no camera is believed. A tilted chassis points its cameras
  // somewhere other than where the Limelight thinks they are, so the floor projection is wrong even
  // though the tags themselves look fine. Raise this if the robot rides rough enough that valid
  // measurements are being thrown away; lower it if bumps are corrupting the pose.
  public static final double MAX_TILT_DEGREES = 5.0;

  // Spin rate beyond which no camera is believed, because the frame is smeared.
  public static final double MAX_YAW_RATE_DEG_PER_SEC = 720.0;

  // Builds the vision configuration. Every supplier comes from the drivetrain -- see StateMachine
  // for what gets wired in and why.
  public static Vision.Config config(
      DoubleSupplier headingDegrees,
      DoubleSupplier pitchDegrees,
      DoubleSupplier rollDegrees,
      DoubleSupplier yawRateDegPerSec,
      Supplier<Pose2d> currentPose) {

    return new Vision.Config(
        // All three cameras, in no particular order -- selection is by score, not by position.
        List.of(
            new Vision.CameraConfig(LEFT_TABLE, LEFT_TRUST),
            new Vision.CameraConfig(RIGHT_TABLE, RIGHT_TRUST),
            new Vision.CameraConfig(TURRET_TABLE, TURRET_TRUST)),
        headingDegrees,
        pitchDegrees,
        rollDegrees,
        yawRateDegPerSec,
        currentPose,
        // Field extents come from FieldConstants so there is one source of truth for them.
        new Vision.FieldBounds(
            FieldConstants.X_MIN,
            FieldConstants.Y_MIN,
            FieldConstants.X_MAX,
            FieldConstants.Y_MAX,
            FIELD_TOLERANCE_METERS),
        // Standard gates, with the two motion limits above substituted in.
        new Vision.AcceptanceParams(
            Vision.AcceptanceParams.defaults().maxSingleTagDist(),
            Vision.AcceptanceParams.defaults().maxMultiTagDist(),
            Vision.AcceptanceParams.defaults().maxSingleTagAmbiguity(),
            Vision.AcceptanceParams.defaults().minSingleTagArea(),
            Vision.AcceptanceParams.defaults().maxJumpSingleTag(),
            MAX_TILT_DEGREES,
            MAX_YAW_RATE_DEG_PER_SEC),
        Vision.StdDevParams.defaults(),
        FUSE_ALL_CAMERAS,
        true);
  }
}
