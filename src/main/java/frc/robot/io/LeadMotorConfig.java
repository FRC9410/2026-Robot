// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

/**
 * Lead (leader) motor configuration: PID and feedback ratios for the primary position motor.
 * Use with {@link CancoderConfig} and {@link MotionMagicConfig} for full position setup.
 */
public record LeadMotorConfig(
    double kP,
    double kI,
    double kD,
    double kG,
    double sensorToMechanismRatio,
    double rotorToSensorRatio) {}
