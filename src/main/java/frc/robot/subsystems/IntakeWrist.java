// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.EncoderHelpers;
import frc.robot.util.MotorCancoderRequest;

/** Subsystem for intake wrist/pivot (raises and lowers intake) using Motion Magic. */
public class IntakeWrist extends SubsystemBase {

  private final TalonFX primaryMotor;
  private final TalonFX secondaryMotor;

  public IntakeWrist() {
    MotorCancoderRequest request =
        MotorCancoderRequest.withDefaults(
            Constants.IntakeConstants.PRIMARY_PIVOT_CAN_ID,
            Constants.IntakeConstants.ENCODER_CAN_ID,
            Constants.IntakeConstants.WRIST_KP,
            Constants.IntakeConstants.WRIST_KI,
            Constants.IntakeConstants.WRIST_KD,
            Constants.IntakeConstants.WRIST_KG,
            Constants.IntakeConstants.WRIST_SENSOR_TO_MECHANISM_RATIO,
            Constants.IntakeConstants.WRIST_ROTOR_TO_SENSOR_RATIO,
            Constants.IntakeConstants.WRIST_MM_CRUISE_VELOCITY,
            Constants.IntakeConstants.WRIST_MM_ACCELERATION);
    primaryMotor = EncoderHelpers.initMotorCancoderPair(request);

    secondaryMotor =
        new TalonFX(
            Constants.IntakeConstants.SECONDARY_PIVOT_CAN_ID,
            Constants.CanBusConstants.CANIVORE_BUS);
    secondaryMotor.setControl(
        new Follower(Constants.IntakeConstants.PRIMARY_PIVOT_CAN_ID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {}

  /** Set wrist position setpoint (rotations) using Motion Magic. */
  public void setPositionRotations(double rotations) {
    primaryMotor.setControl(new MotionMagicVoltage(0).withPosition(rotations).withSlot(0));
  }

  /** Set wrist position setpoint in degrees using Motion Magic. */
  public void setPositionDegrees(double degrees) {
    setPositionRotations(degrees / 360.0);
  }

  /** Move wrist to down (intake) setpoint. */
  public void setDown() {
    setPositionRotations(Constants.IntakeConstants.INTAKE_DOWN_SETPOINT);
  }

  /** Move wrist to up (stow) setpoint. */
  public void setUp() {
    setPositionRotations(Constants.IntakeConstants.INTAKE_UP_SETPOINT);
  }

  /** Set pivot motor output (e.g. for manual control). */
  public void setOutput(double output) {
    primaryMotor.set(output);
  }

  /** Stop the wrist motors. */
  public void stop() {
    primaryMotor.set(0);
  }

  /** Get wrist position in rotations (from fused CANcoder). */
  public double getPositionRotations() {
    return primaryMotor.getPosition().getValueAsDouble();
  }

  /** Get wrist position in degrees. */
  public double getPositionDegrees() {
    return getPositionRotations() * 360.0;
  }

  public TalonFX getPrimaryMotor() {
    return primaryMotor;
  }

  public TalonFX getSecondaryMotor() {
    return secondaryMotor;
  }
}
