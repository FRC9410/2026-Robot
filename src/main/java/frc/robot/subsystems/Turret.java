// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorCancoderRequest;

/** Subsystem for turret rotation (yaw) using Motion Magic profiler. */
public class Turret extends SubsystemBase {

  private final TalonFX motor;

  public Turret() {
    MotorCancoderRequest request =
        MotorCancoderRequest.withDefaults(
            Constants.TurretConstants.MOTOR_ID,
            Constants.TurretConstants.ENCODER_ID,
            Constants.TurretConstants.TURRET_KP,
            Constants.TurretConstants.TURRET_KI,
            Constants.TurretConstants.TURRET_KD,
            Constants.TurretConstants.TURRET_KG,
            Constants.TurretConstants.TURRET_SENSOR_TO_MECHANISM_RATIO,
            Constants.TurretConstants.TURRET_ROTOR_TO_SENSOR_RATIO,
            Constants.TurretConstants.TURRET_MM_CRUISE_VELOCITY,
            Constants.TurretConstants.TURRET_MM_ACCELERATION);
    motor = PositionSubsystem.initMotorCancoderPair(request);
  }

  @Override
  public void periodic() {}

  /** Set turret position setpoint (rotations) using Motion Magic profiler. */
  public void setPositionRotations(double rotations) {
    motor.setControl(new MotionMagicVoltage(0).withPosition(rotations).withSlot(0));
  }

  /** Set turret position setpoint in degrees using Motion Magic profiler. */
  public void setPositionDegrees(double degrees) {
    setPositionRotations(degrees / 360.0);
  }

  /** Set raw output (e.g. for manual control). */
  public void setOutput(double output) {
    motor.set(output);
  }

  /** Stop the turret motor. */
  public void stop() {
    motor.set(0);
  }

  /** Get current position from fused CANcoder (rotations). */
  public double getPositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  /** Get current position in degrees. */
  public double getPositionDegrees() {
    return getPositionRotations() * 360.0;
  }

  public TalonFX getMotor() {
    return motor;
  }
}
