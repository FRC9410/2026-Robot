package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.io.CancoderConfig;
import frc.robot.io.LeadMotorConfig;
import frc.robot.io.MotionMagicConfig;
import frc.robot.io.MotorConfig;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    /** Roller velocity setpoint (RPS) for intake/outtake when using VelocitySubsystem. */
    public static final double ROLLER_TARGET_RPS = 10;

    /** Roller velocity PID (Slot0). Tune as needed. */
    public static final double ROLLER_KP = 0.1;
    public static final double ROLLER_KI = 0;
    public static final double ROLLER_KD = 0;
    public static final double ROLLER_KG = 0;
    public static final double ROLLER_MM_ACCELERATION = 30;

    /** Motor config for intake roller (single leader, coast). */
    public static final List<MotorConfig> ROLLER_MOTOR_CONFIGS = List.of(
        MotorConfig.leader(ROLLER_CAN_ID, NeutralModeValue.Coast));

    /** Lead motor config for roller velocity control. */
    public static final LeadMotorConfig ROLLER_LEAD_CONFIG = new LeadMotorConfig(
        ROLLER_KP, ROLLER_KI, ROLLER_KD, ROLLER_KG, 1.0, 1.0);

    /** Motion Magic config for roller velocity. */
    public static final MotionMagicConfig ROLLER_MOTION_MAGIC_CONFIG =
        MotionMagicConfig.forVelocity(ROLLER_MM_ACCELERATION);

    /** Wrist PID (Slot0). Tune as needed. */
    public static final double WRIST_KP = 20;
    public static final double WRIST_KI = 0;
    public static final double WRIST_KD = 0;
    public static final double WRIST_KG = 0;
    public static final double WRIST_SENSOR_TO_MECHANISM_RATIO = 1;
    public static final double WRIST_ROTOR_TO_SENSOR_RATIO = 1;
    public static final double WRIST_MM_CRUISE_VELOCITY = 20;
    public static final double WRIST_MM_ACCELERATION = 40;
    /** CANcoder magnet offset and discontinuity (rotations). */
    public static final double WRIST_MAGNET_OFFSET_ROTATIONS = -0.1;
    public static final double WRIST_DISCONTINUITY_POINT_ROTATIONS = 1.0;

    /** Motor configs for wrist: leader (primary pivot) + follower (secondary pivot). */
    public static final List<MotorConfig> WRIST_MOTOR_CONFIGS = List.of(
        MotorConfig.leader(PRIMARY_PIVOT_CAN_ID, NeutralModeValue.Brake),
        MotorConfig.follower(SECONDARY_PIVOT_CAN_ID));

    /** Lead motor config for wrist position control. */
    public static final LeadMotorConfig WRIST_LEAD_CONFIG = new LeadMotorConfig(
        WRIST_KP, WRIST_KI, WRIST_KD, WRIST_KG,
        WRIST_SENSOR_TO_MECHANISM_RATIO, WRIST_ROTOR_TO_SENSOR_RATIO);

    /** CANcoder config for wrist. */
    public static final CancoderConfig WRIST_CANCODER_CONFIG = new CancoderConfig(
        ENCODER_CAN_ID, WRIST_MAGNET_OFFSET_ROTATIONS, WRIST_DISCONTINUITY_POINT_ROTATIONS);

    /** Motion Magic config for wrist. */
    public static final MotionMagicConfig WRIST_MOTION_MAGIC_CONFIG = new MotionMagicConfig(
        WRIST_MM_CRUISE_VELOCITY, WRIST_MM_ACCELERATION);
  }

  public static final class SpindexerConstants {
    public static final int CAN_ID = 30;
    public static final int LASER_1_CAN_ID = 31;
    public static final int LASER_2_CAN_ID = 32;

    /** Spindexer velocity PID (Slot0). Tune as needed. */
    public static final double SPINDEXER_KP = 0.1;
    public static final double SPINDEXER_KI = 0;
    public static final double SPINDEXER_KD = 0;
    public static final double SPINDEXER_KG = 0;
    public static final double SPINDEXER_MM_ACCELERATION = 30;

    /** Motor config for spindexer (single leader, brake). */
    public static final List<MotorConfig> SPINDEXER_MOTOR_CONFIGS = List.of(
        MotorConfig.leader(CAN_ID, NeutralModeValue.Brake));

    /** Lead motor config for spindexer velocity control. */
    public static final LeadMotorConfig SPINDEXER_LEAD_CONFIG = new LeadMotorConfig(
        SPINDEXER_KP, SPINDEXER_KI, SPINDEXER_KD, SPINDEXER_KG, 1.0, 1.0);

    /** Motion Magic config for spindexer velocity. */
    public static final MotionMagicConfig SPINDEXER_MOTION_MAGIC_CONFIG =
        MotionMagicConfig.forVelocity(SPINDEXER_MM_ACCELERATION);
  }

  public static final class FeederConstants {
    public static final int PRIMARY_CAN_ID = 40;
    public static final int SECONDARY_CAN_ID = 41;
    public static final int CANDI1_CAN_ID = 42;
    public static final int CANDI2_CAN_ID = 43;

    /** Feeder velocity PID (Slot0). Tune as needed. */
    public static final double FEEDER_KP = 0.1;
    public static final double FEEDER_KI = 0;
    public static final double FEEDER_KD = 0;
    public static final double FEEDER_KG = 0;
    public static final double FEEDER_MM_ACCELERATION = 40;

    public static final List<MotorConfig> MOTOR_CONFIGS = List.of(
      MotorConfig.leader(PRIMARY_CAN_ID, NeutralModeValue.Coast),
      MotorConfig.follower(SECONDARY_CAN_ID)
    );

    /** Lead motor config for feeder velocity control. */
    public static final LeadMotorConfig FEEDER_LEAD_CONFIG = new LeadMotorConfig(
        FEEDER_KP, FEEDER_KI, FEEDER_KD, FEEDER_KG, 1.0, 1.0);

    /** Motion Magic config for feeder velocity. */
    public static final MotionMagicConfig FEEDER_MOTION_MAGIC_CONFIG =
        MotionMagicConfig.forVelocity(FEEDER_MM_ACCELERATION);
  }
  public static final class ShooterConstants {
    public static final int HOOD_CAN_ID = 50;
    public static final int HOOD_ENCODER_CAN_ID = 53;
    public static final int PRIMARY_FLYWHEELS_CAN_ID = 51;
    public static final int SECONDARY_FLYWHEELS_CAN_ID = 52;
    public static final int ENCODER_CAN_ID = 53;

    /** Flywheel velocity PID (Slot0). Tune as needed. */
    public static final double FLYWHEEL_KP = 0.1;
    public static final double FLYWHEEL_KI = 0;
    public static final double FLYWHEEL_KD = 0;
    public static final double FLYWHEEL_KG = 0;
    /** Motion Magic acceleration for flywheel velocity. */
    public static final double FLYWHEEL_MM_ACCELERATION = 50;

    /** Motor config for flywheels: leader (primary) + follower (secondary), coast. */
    public static final List<MotorConfig> FLYWHEEL_MOTOR_CONFIGS = List.of(
        MotorConfig.leader(PRIMARY_FLYWHEELS_CAN_ID, NeutralModeValue.Coast),
        MotorConfig.follower(SECONDARY_FLYWHEELS_CAN_ID));

    /** Lead motor config for flywheel velocity control. */
    public static final LeadMotorConfig FLYWHEEL_LEAD_CONFIG = new LeadMotorConfig(
        FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KG, 1.0, 1.0);

    /** Motion Magic config for flywheel velocity. */
    public static final MotionMagicConfig FLYWHEEL_MOTION_MAGIC_CONFIG =
        MotionMagicConfig.forVelocity(FLYWHEEL_MM_ACCELERATION);

    /** Hood PID (Slot0). Tune as needed. */
    public static final double HOOD_KP = 20;
    public static final double HOOD_KI = 0;
    public static final double HOOD_KD = 0;
    public static final double HOOD_KG = 0;
    public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = 1;
    public static final double HOOD_ROTOR_TO_SENSOR_RATIO = 1;
    public static final double HOOD_MM_CRUISE_VELOCITY = 20;
    public static final double HOOD_MM_ACCELERATION = 40;
    /** CANcoder magnet offset and discontinuity (rotations). */
    public static final double HOOD_MAGNET_OFFSET_ROTATIONS = -0.1;
    public static final double HOOD_DISCONTINUITY_POINT_ROTATIONS = 1.0;

    /** Motor config for hood (single leader). */
    public static final List<MotorConfig> HOOD_MOTOR_CONFIGS = List.of(
        MotorConfig.leader(HOOD_CAN_ID, NeutralModeValue.Brake));

    /** Lead motor config for hood position control. */
    public static final LeadMotorConfig HOOD_LEAD_CONFIG = new LeadMotorConfig(
        HOOD_KP, HOOD_KI, HOOD_KD, HOOD_KG,
        HOOD_SENSOR_TO_MECHANISM_RATIO, HOOD_ROTOR_TO_SENSOR_RATIO);

    /** CANcoder config for hood. */
    public static final CancoderConfig HOOD_CANCODER_CONFIG = new CancoderConfig(
        HOOD_ENCODER_CAN_ID, HOOD_MAGNET_OFFSET_ROTATIONS, HOOD_DISCONTINUITY_POINT_ROTATIONS);

    /** Motion Magic config for hood. */
    public static final MotionMagicConfig HOOD_MOTION_MAGIC_CONFIG = new MotionMagicConfig(
        HOOD_MM_CRUISE_VELOCITY, HOOD_MM_ACCELERATION);
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
    public static final double TURRET_KP = 20;
    public static final double TURRET_KI = 0;
    public static final double TURRET_KD = 0;
    public static final double TURRET_KG = 0;

    /** Feedback ratios (sensor/rotor to mechanism). Adjust for your gearbox/encoder. */
    public static final double TURRET_SENSOR_TO_MECHANISM_RATIO = 1;
    public static final double TURRET_ROTOR_TO_SENSOR_RATIO = 1;

    /** Motion Magic profiler. */
    public static final double TURRET_MM_CRUISE_VELOCITY = 20;
    public static final double TURRET_MM_ACCELERATION = 40;
    /** CANcoder magnet offset and discontinuity (rotations). */
    public static final double TURRET_MAGNET_OFFSET_ROTATIONS = -0.1;
    public static final double TURRET_DISCONTINUITY_POINT_ROTATIONS = 1.0;

    /** Motor config for turret (single leader). */
    public static final List<MotorConfig> TURRET_MOTOR_CONFIGS = List.of(
        MotorConfig.leader(MOTOR_ID, NeutralModeValue.Brake));

    /** Lead motor config for turret position control. */
    public static final LeadMotorConfig TURRET_LEAD_CONFIG = new LeadMotorConfig(
        TURRET_KP, TURRET_KI, TURRET_KD, TURRET_KG,
        TURRET_SENSOR_TO_MECHANISM_RATIO, TURRET_ROTOR_TO_SENSOR_RATIO);

    /** CANcoder config for turret. */
    public static final CancoderConfig TURRET_CANCODER_CONFIG = new CancoderConfig(
        ENCODER_ID, TURRET_MAGNET_OFFSET_ROTATIONS, TURRET_DISCONTINUITY_POINT_ROTATIONS);

    /** Motion Magic config for turret. */
    public static final MotionMagicConfig TURRET_MOTION_MAGIC_CONFIG = new MotionMagicConfig(
        TURRET_MM_CRUISE_VELOCITY, TURRET_MM_ACCELERATION);

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
