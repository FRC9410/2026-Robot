// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for feeder (transfers game pieces to shooter; optional CANDI/beam sensors). */
public class Feeder extends SubsystemBase {

  private final TalonFX primaryMotor;
  private final TalonFX secondaryMotor;

  public Feeder() {
    primaryMotor =
        new TalonFX(
            Constants.FeederConstants.PRIMARY_CAN_ID,
            Constants.CanBusConstants.CANIVORE_BUS);
    secondaryMotor =
        new TalonFX(
            Constants.FeederConstants.SECONDARY_CAN_ID,
            Constants.CanBusConstants.CANIVORE_BUS);

    primaryMotor.setNeutralMode(NeutralModeValue.Coast);
    secondaryMotor.setNeutralMode(NeutralModeValue.Coast);
    secondaryMotor.setControl(
        new Follower(Constants.FeederConstants.PRIMARY_CAN_ID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {}

  /** Set feeder output (positive = feed toward shooter). */
  public void setOutput(double output) {
    primaryMotor.set(output);
  }

  /** Stop the feeder motors. */
  public void stop() {
    primaryMotor.set(0);
  }

  /** Whether note is detected at sensor 1 (placeholder; wire CANDI when available). */
  public boolean hasNoteAtSensor1() {
    return false; // TODO: use FeederConstants.CANDI1_CAN_ID
  }

  /** Whether note is detected at sensor 2 (placeholder; wire CANDI when available). */
  public boolean hasNoteAtSensor2() {
    return false; // TODO: use FeederConstants.CANDI2_CAN_ID
  }

  public TalonFX getPrimaryMotor() {
    return primaryMotor;
  }

  public TalonFX getSecondaryMotor() {
    return secondaryMotor;
  }
}
