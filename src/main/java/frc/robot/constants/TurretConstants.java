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
    public static final double TURRET_MAGNET_OFFSET_ROTATIONS = 0.1315;
    public static final double TURRET_DISCONTINUITY_POINT_ROTATIONS = 0.5;

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

    public static final double[][] HOOD_ANGLES = {
            { 4.35, 0.12 },
            { 4.3, 0.11 },
            { 4.1, 0.1 },
            { 3.75, 0.09 },
            { 3.45, 0.08 },
            { 3.13, 0.07 },
            { 2.8, 0.065 },
            { 2.55, 0.06 },
            { 2.22, 0.055 },
            { 1.95, 0.055 },
            { 1.67, 0.05 }
    };

    public static final double[][] SHOOTER_SPEEDS = {
            { 4.35, 120 },
            { 4.3, 105 },
            { 4.1, 100 },
            { 3.75, 100 },
            { 3.45, 95 },
            { 3.13, 95 },
            { 2.8, 95 },
            { 2.55, 93 },
            { 2.22, 87 },
            { 1.95, 83 },
            { 1.67, 80 }
    };

    public static final double[][] MIN_VELO_TABLE = {
        {4.5, -88},
        {4.0, -75},
        {3.6, 75},
        {3.25, 72},
        {2.8, 72}, 
        {2.5, 70},
        {2.25, 65},
        {2.0, 63}
    };

    public static final double TURRET_DIST_FROM_ROBOT_CENTER = -0.1778;
    public static final double TURRET_RADIUS = 0.10795; //.127

    public static final LinearInterpolator HOOD_ANGLE_INTERPOLATOR = new LinearInterpolator(HOOD_ANGLES);
    public static final LinearInterpolator SHOOTER_VELOCITY_INTERPOLATOR = new LinearInterpolator(SHOOTER_SPEEDS);
    public static final LinearInterpolator MIN_VELO_INTERPOLATOR = new LinearInterpolator(MIN_VELO_TABLE);

    public static final double TURRET_CAMERA_Y_OFFSET = 0.0;
}
