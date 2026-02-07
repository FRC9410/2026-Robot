// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for turret rotation (yaw). */
public class Turret extends SubsystemBase {

  private final TalonFX motor;
  private final CANcoder encoder;

  public Turret() {
    motor =
        new TalonFX(
            Constants.TurretConstants.MOTOR_ID,
            Constants.CanBusConstants.CANIVORE_BUS);
    encoder =
        new CANcoder(
            Constants.TurretConstants.ENCODER_ID,
            Constants.CanBusConstants.CANIVORE_BUS);

    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {}

  /** Set turret rotation output (e.g. from a PID or manual control). */
  public void setOutput(double output) {
    motor.set(output);
  }

  /** Stop the turret motor. */
  public void stop() {
    motor.set(0);
  }

  /** Get current position from the CANcoder (rotations). */
  // public double getPositionRotations() {
  //   return encoder.getPosition().getValue();
  // }

  /** Get current position in degrees. */
  // public double getPositionDegrees() {
  //   return getPositionRotations() * 360.0;
  // }

  public TalonFX getMotor() {
    return motor;
  }

  public CANcoder getEncoder() {
    return encoder;
  }
}
