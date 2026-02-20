package frc.robot.constants;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team9410.configs.CancoderConfig;
import frc.lib.team9410.configs.LeadMotorConfig;
import frc.lib.team9410.configs.MotionMagicConfig;
import frc.lib.team9410.configs.MotorConfig;
import frc.lib.team9410.configs.PositionSubsystemConfig;
import frc.lib.team9410.math.LinearInterpolator;

public class TurretConstants {
  public static final int MOTOR_ID = 60;
  public static final int ENCODER_ID = 61;

  public static final double TURRET_KP = 54;
  public static final double TURRET_KI = 0;
  public static final double TURRET_KD = 0;
  public static final double TURRET_KG = 0;

  public static final double TURRET_MIN = -0.481;
  public static final double TURRET_MAX = 0.46;
  public static final double TURRET_DEFAULT = 0.0;

  public static final double TURRET_SENSOR_TO_MECHANISM_RATIO = -1;
  public static final double TURRET_ROTOR_TO_SENSOR_RATIO = 51;

  public static final double TURRET_MM_CRUISE_VELOCITY = 5;
  public static final double TURRET_MM_ACCELERATION = 60;
  public static final double TURRET_MAGNET_OFFSET_ROTATIONS = 0.701;
  public static final double TURRET_DISCONTINUITY_POINT_ROTATIONS = 1.0;

  public static final List<MotorConfig> TURRET_MOTOR_CONFIGS = List.of(
      MotorConfig.leader(MOTOR_ID, NeutralModeValue.Brake, true));

  public static final LeadMotorConfig TURRET_LEAD_CONFIG = new LeadMotorConfig(
      TURRET_KP, TURRET_KI, TURRET_KD, TURRET_KG,
      TURRET_SENSOR_TO_MECHANISM_RATIO, TURRET_ROTOR_TO_SENSOR_RATIO);

  public static final CancoderConfig TURRET_CANCODER_CONFIG = new CancoderConfig(
      ENCODER_ID, TURRET_MAGNET_OFFSET_ROTATIONS, TURRET_DISCONTINUITY_POINT_ROTATIONS);

  public static final MotionMagicConfig TURRET_MOTION_MAGIC_CONFIG = new MotionMagicConfig(
      TURRET_MM_CRUISE_VELOCITY, TURRET_MM_ACCELERATION);

  public static final PositionSubsystemConfig TURRET_CONFIG = new PositionSubsystemConfig(
      TURRET_MOTOR_CONFIGS, TURRET_LEAD_CONFIG, TURRET_CANCODER_CONFIG, TURRET_MOTION_MAGIC_CONFIG,
      "Turret", "degrees", Optional.of(TURRET_DEFAULT));

  public static final LinearInterpolator shooterVelocityInterpolator = new LinearInterpolator(ShooterConstants.shooterSpeeds);
  public static final LinearInterpolator hoodAngleInterpolator = new LinearInterpolator(ShooterConstants.hoodAngles);

  

  public static final int TURRET_DIST_FROM_ROBOT_CENTER = 0;
  public static final int TURRET_RADIUS = 0;
}
