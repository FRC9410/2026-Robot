package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.awt.geom.Point2D;
import java.util.List;

public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0; // Typically, USB port 0
    public static final int OPERATOR_CONTROLLER_PORT = 1; // Typically, USB port 1
    // Deadband constant for joystick/controller input (placeholder, adjust as needed)
    public static final double DEADBAND = 0.05;
  }

  public static final class CanBusConstants {
    public static final String CANIVORE_BUS = "canivore";
  }

  public static final class AutoConstants {
    public static final double TRANSLATION_TOLERANCE = 0.05;
    public static final double ROTATION_TOLERANCE = 1.0;

    public static final Pose2d TEST_POSITION = new Pose2d(5, 0, Rotation2d.fromDegrees(90.0));
    public static final Pose2d TEST_POSITION2 = new Pose2d(0, 0.0254, Rotation2d.fromDegrees(0.0));
    public static final Pose2d TEST_POSITION3 = new Pose2d(0, -0.0254, Rotation2d.fromDegrees(0.0));
  }

  public static final class VisionConstants {
    // Vision system constants (placeholders—adjust as needed)
    public static final String CAMERA_NAME = "limelight";
    public static final double CAMERA_FOV_DEGREES = 60.0;
    public static final int IMAGE_WIDTH = 320;
    public static final int IMAGE_HEIGHT = 240;
    // Example threshold and distance constants
    public static final double TARGET_AREA_THRESHOLD = 1.0;
    public static final double MAX_TARGET_DISTANCE_METERS = 5.0;

    // Additional Vision constants for Reef Vision:
    public static final String LEFT_TABLE = "limelight-left"; // Placeholder table name
    public static final String RIGHT_TABLE = "limelight-right"; // Placeholder table name
  }


  public static final class FieldConstants {
    public static final double X_MIN = 0.0;
    public static final double Y_MIN = 0.0;
    public static final double X_MAX = 17.5;
    public static final double Y_MAX = 8.0;
    public static final double TOL = 2.0;
  }

}
