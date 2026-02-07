package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix6.CANBus;

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
    public static final CANBus CANIVORE_BUS = new CANBus("canivore");
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

  public static final class IntakeConstants {
    public static final int ROLLER_CAN_ID = 20;
    public static final int PRIMARY_PIVOT_CAN_ID = 21;
    public static final int SECONDARY_PIVOT_CAN_ID = 22;
    public static final int ENCODER_CAN_ID = 23;

    public static final double INTAKE_DOWN_SETPOINT = 0; // placeholder rotations
    public static final double INTAKE_UP_SETPOINT = 0; // placeholder rotations
    /** Roller percent output for intake (e.g. 0.5–1.0); tune as needed. */
    public static final double INTAKE_ROLLER_OUTPUT = 0.8;

    /** Wrist PID (Slot0). Tune as needed. */
    public static final int WRIST_KP = 20;
    public static final int WRIST_KI = 0;
    public static final int WRIST_KD = 0;
    public static final int WRIST_KG = 0;
    public static final int WRIST_SENSOR_TO_MECHANISM_RATIO = 1;
    public static final int WRIST_ROTOR_TO_SENSOR_RATIO = 1;
    public static final int WRIST_MM_CRUISE_VELOCITY = 20;
    public static final int WRIST_MM_ACCELERATION = 40;
  }

  public static final class SpindexerConstants {
    public static final int CAN_ID = 30;
    public static final int LASER_1_CAN_ID = 31;
    public static final int LASER_2_CAN_ID = 32;
  }

  public static final class FeederConstants {
    public static final int PRIMARY_CAN_ID = 40;
    public static final int SECONDARY_CAN_ID = 41;
    public static final int CANDI1_CAN_ID = 42;
    public static final int CANDI2_CAN_ID = 43;
  }
  public static final class ShooterConstants {
    public static final int HOOD_CAN_ID = 50;
    public static final int HOOD_ENCODER_CAN_ID = 53;
    public static final int PRIMARY_FLYWHEELS_CAN_ID = 51;
    public static final int SECONDARY_FLYWHEELS_CAN_ID = 52;
    public static final int ENCODER_CAN_ID = 53;

    /** Hood PID (Slot0). Tune as needed. */
    public static final int HOOD_KP = 20;
    public static final int HOOD_KI = 0;
    public static final int HOOD_KD = 0;
    public static final int HOOD_KG = 0;
    public static final int HOOD_SENSOR_TO_MECHANISM_RATIO = 1;
    public static final int HOOD_ROTOR_TO_SENSOR_RATIO = 1;
    public static final int HOOD_MM_CRUISE_VELOCITY = 20;
    public static final int HOOD_MM_ACCELERATION = 40;
  }

  public static final class LEDConstants {
    /** CAN ID of the CTRE CANdle. */
    public static final int CANDLE_CAN_ID = 0;
    /** Number of LEDs (onboard 0–7 + strip; use 8 to 8+N-1 for strip only). */
    public static final int STRIP_LENGTH = 60;
    /** First LED index for the strip on the CANdle (8 = first strip LED). */
    public static final int STRIP_START_INDEX = 8;
  }

  public static final class TurretConstants {
    public static final int MOTOR_ID = 60;
    public static final int ENCODER_ID = 61;

    /** PID gains for turret (Slot0). Tune as needed. */
    public static final int TURRET_KP = 20;
    public static final int TURRET_KI = 0;
    public static final int TURRET_KD = 0;
    public static final int TURRET_KG = 0;

    /** Feedback ratios (sensor/rotor to mechanism). Adjust for your gearbox/encoder. */
    public static final int TURRET_SENSOR_TO_MECHANISM_RATIO = 1;
    public static final int TURRET_ROTOR_TO_SENSOR_RATIO = 1;

    /** Motion Magic profiler. */
    public static final int TURRET_MM_CRUISE_VELOCITY = 20;
    public static final int TURRET_MM_ACCELERATION = 40;

    // TODO: fix placeholder values
    public static final int TURRET_DIST_FROM_ROBOT_CENTER = 0; // in inches
    public static final int TURRET_RADIUS = 0; // in inches
  }

  public static final class FieldConstants {
    public static final double X_MIN = 0.0;
    public static final double Y_MIN = 0.0;
    public static final double X_MAX = 17.5;
    public static final double Y_MAX = 8.0;
    public static final double TOL = 2.0;
  }

}
