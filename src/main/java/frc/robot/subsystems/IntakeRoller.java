// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for intake roller (grabs game pieces). */
public class IntakeRoller extends SubsystemBase {

  private final TalonFX motor;

  public IntakeRoller() {
    motor =
        new TalonFX(
            Constants.IntakeConstants.ROLLER_CAN_ID,
            Constants.CanBusConstants.CANIVORE_BUS);

    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {}

  /** Set roller output (positive = intake). */
  public void setOutput(double output) {
    motor.set(output);
  }

  /** Run roller at configured intake output. */
  public void runIntake() {
    setOutput(Constants.IntakeConstants.INTAKE_ROLLER_OUTPUT);
  }

  /** Run roller in reverse (e.g. outtake). */
  public void runOuttake() {
    setOutput(-Constants.IntakeConstants.INTAKE_ROLLER_OUTPUT);
  }

  /** Stop the roller. */
  public void stop() {
    motor.set(0);
  }

  public TalonFX getMotor() {
    return motor;
  }
}
