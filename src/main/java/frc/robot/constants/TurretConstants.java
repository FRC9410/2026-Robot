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

    public static final double TURRET_KP = 57;
    public static final double TURRET_KI = 0;
    public static final double TURRET_KD = 0;
    public static final double TURRET_KG = 0;

    public static final double TURRET_MIN = -0.481;
    public static final double TURRET_MAX = 0.46;
    public static final double TURRET_DEFAULT = 0.0;

    public static final double TURRET_SENSOR_TO_MECHANISM_RATIO = -1;
    public static final double TURRET_ROTOR_TO_SENSOR_RATIO = 51 * (8.5 / 9);

    public static final double TURRET_MM_CRUISE_VELOCITY = 3; // 5
    public static final double TURRET_MM_ACCELERATION = 60; // 60
    public static final double TURRET_MAGNET_OFFSET_ROTATIONS = 0.1255;
    public static final double TURRET_DISCONTINUITY_POINT_ROTATIONS = 0.5;

    public static final List<MotorConfig> TURRET_MOTOR_CONFIGS = List.of(
            MotorConfig.leader(MOTOR_ID, NeutralModeValue.Brake, true));

    public static final LeadMotorConfig TURRET_LEAD_CONFIG = new LeadMotorConfig(
            TURRET_KP, TURRET_KI, TURRET_KD, TURRET_KG, Optional.empty(), Optional.empty(), Optional.empty(),
            TURRET_SENSOR_TO_MECHANISM_RATIO, TURRET_ROTOR_TO_SENSOR_RATIO);

    public static final CancoderConfig TURRET_CANCODER_CONFIG = new CancoderConfig(
            ENCODER_ID, TURRET_MAGNET_OFFSET_ROTATIONS, TURRET_DISCONTINUITY_POINT_ROTATIONS);

    public static final MotionMagicConfig TURRET_MOTION_MAGIC_CONFIG = new MotionMagicConfig(
            TURRET_MM_CRUISE_VELOCITY, TURRET_MM_ACCELERATION);

    public static final PositionSubsystemConfig TURRET_CONFIG = new PositionSubsystemConfig(
            TURRET_MOTOR_CONFIGS, TURRET_LEAD_CONFIG, TURRET_CANCODER_CONFIG, TURRET_MOTION_MAGIC_CONFIG,
            "Turret", "degrees", Optional.of(TURRET_DEFAULT));

    public static final double[][] HOOD_ANGLES = {
            { 5.5, 0.135 },
            { 4.85, 0.105 },
            { 4.6, 0.09 },
            { 4.05, 0.08 },
            { 3.75, 0.07 },
            { 3.29, 0.06 },
            { 2.75, 0.05 },
            { 2.38, 0.04 },
            { 1.97, 0.03 },
            { 1.68, 0.02 }
    };

    public static final double[][] SHOOTER_SPEEDS = {
             { 5.5, 89},
            { 4.85, 84},
            { 4.6, 82},
            { 4.05, 79},
            { 3.75, 77},
            { 3.29, 74},
            { 2.75, 71},
            { 2.38, 67},
            { 1.97, 64},
            { 1.68, 62}
    };

    public static final double[][] FEEDER_SPEEDS = {
        { 5.5, 72},
        { 4.85, 68},
        { 4.6, 67},
        { 4.05, 66},
        { 3.75, 65},
        { 3.29, 65},
        { 2.75, 63},
        { 2.38, 61},
        { 1.97, 59},
        { 1.68, 58}
    };

    public static final double TURRET_DIST_FROM_ROBOT_CENTER = -0.1778;
    public static final double TURRET_RADIUS = 0.10795; //.127

    /** Max turret angle error (degrees) from target before allowing shoot (must be within this to fire). */
    public static final double TURRET_SHOOT_ANGLE_TOLERANCE_DEG = 2.0;

    /** Max drivetrain linear speed (m/s) to allow shooting; above this we brake and do not shoot. */
    public static final double SHOOT_MAX_DRIVETRAIN_SPEED_MPS = 0.5;

    public static final LinearInterpolator HOOD_ANGLE_INTERPOLATOR = new LinearInterpolator(HOOD_ANGLES);
    public static final LinearInterpolator SHOOTER_VELOCITY_INTERPOLATOR = new LinearInterpolator(SHOOTER_SPEEDS);
    public static final LinearInterpolator FEEDER_VELOCITY_INTERPOLATOR = new LinearInterpolator(FEEDER_SPEEDS);

    public static final double TURRET_CAMERA_Y_OFFSET = 0.0;
}
