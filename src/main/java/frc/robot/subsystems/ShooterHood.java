// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.EncoderHelpers;
import frc.robot.util.MotorCancoderRequest;

/** Subsystem for shooter hood angle (elevation) using Motion Magic. */
public class ShooterHood extends SubsystemBase {

  private final TalonFX motor;

  public ShooterHood() {
    MotorCancoderRequest request =
        MotorCancoderRequest.withDefaults(
            Constants.ShooterConstants.HOOD_CAN_ID,
            Constants.ShooterConstants.HOOD_ENCODER_CAN_ID,
            Constants.ShooterConstants.HOOD_KP,
            Constants.ShooterConstants.HOOD_KI,
            Constants.ShooterConstants.HOOD_KD,
            Constants.ShooterConstants.HOOD_KG,
            Constants.ShooterConstants.HOOD_SENSOR_TO_MECHANISM_RATIO,
            Constants.ShooterConstants.HOOD_ROTOR_TO_SENSOR_RATIO,
            Constants.ShooterConstants.HOOD_MM_CRUISE_VELOCITY,
            Constants.ShooterConstants.HOOD_MM_ACCELERATION);
    motor = EncoderHelpers.initMotorCancoderPair(request);
  }

  @Override
  public void periodic() {}

  /** Set hood position setpoint (rotations) using Motion Magic. */
  public void setPositionRotations(double rotations) {
    motor.setControl(new MotionMagicVoltage(0).withPosition(rotations).withSlot(0));
  }

  /** Set hood position setpoint in degrees using Motion Magic. */
  public void setPositionDegrees(double degrees) {
    setPositionRotations(degrees / 360.0);
  }

  /** Set hood motor output (e.g. for manual control). */
  public void setOutput(double output) {
    motor.set(output);
  }

  /** Stop the hood motor. */
  public void stop() {
    motor.set(0);
  }

  /** Get hood position in rotations (from fused CANcoder). */
  public double getPositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  /** Get hood position in degrees. */
  public double getPositionDegrees() {
    return getPositionRotations() * 360.0;
  }

  public TalonFX getMotor() {
    return motor;
  }
}
