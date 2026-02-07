// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants;

/**
 * Request object for initializing a TalonFX with PID and Motion Magic velocity config.
 * Use with {@link VelocitySubsystem#initFromRequest(VelocityConfigRequest)}.
 */
public record VelocityConfigRequest(
    int canId,
    CANBus canBus,
    double kP,
    double kI,
    double kD,
    double kG,
    double stopVoltage,
    double mmAcceleration) {

  /**
   * Creates a request with the default CAN bus (CANivore).
   */
  public static VelocityConfigRequest withDefaults(
      int canId,
      double kP,
      double kI,
      double kD,
      double kG,
      double stopVoltage,
      double mmAcceleration) {
    return new VelocityConfigRequest(
        canId,
        Constants.CanBusConstants.CANIVORE_BUS,
        kP,
        kI,
        kD,
        kG,
        stopVoltage,
        mmAcceleration);
  }
}
