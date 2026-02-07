// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for shooter hood angle (elevation). */
public class ShooterHood extends SubsystemBase {

  private final TalonFX motor;

  public ShooterHood() {
    motor =
        new TalonFX(
            Constants.ShooterConstants.HOOD_CAN_ID,
            Constants.CanBusConstants.CANIVORE_BUS);

    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {}

  /** Set hood motor output (e.g. from manual control or position PID). */
  public void setOutput(double output) {
    motor.set(output);
  }

  /** Stop the hood motor. */
  public void stop() {
    motor.set(0);
  }

  /** Get hood position in rotations (use for closed-loop when encoder is configured). */
  public double getPositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public TalonFX getMotor() {
    return motor;
  }
}
