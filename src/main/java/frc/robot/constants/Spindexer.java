package frc.robot.constants;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team9410.configs.LeadMotorConfig;
import frc.lib.team9410.configs.MotionMagicConfig;
import frc.lib.team9410.configs.MotorConfig;
import frc.lib.team9410.configs.VelocitySubsystemConfig;

public final class Spindexer {
        public static class Constants {
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
                public static final MotionMagicConfig SPINDEXER_MOTION_MAGIC_CONFIG = MotionMagicConfig
                                .forVelocity(SPINDEXER_MM_ACCELERATION);

                /** Velocity subsystem config for spindexer. */
                public static final VelocitySubsystemConfig SPINDEXER_CONFIG = new VelocitySubsystemConfig(
                                SPINDEXER_MOTOR_CONFIGS, SPINDEXER_LEAD_CONFIG, SPINDEXER_MOTION_MAGIC_CONFIG,
                                "Spindexer");
        }
}
