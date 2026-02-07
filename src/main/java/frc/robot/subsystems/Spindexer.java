// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for spindexer (indexes game pieces; optional laser/beam sensors). */
public class Spindexer extends SubsystemBase {

  private final TalonFX motor;

  public Spindexer() {
    motor =
        new TalonFX(
            Constants.SpindexerConstants.CAN_ID,
            Constants.CanBusConstants.CANIVORE_BUS);

    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {}

  /** Set spindexer output (positive = index forward). */
  public void setOutput(double output) {
    motor.set(output);
  }

  /** Stop the spindexer. */
  public void stop() {
    motor.set(0);
  }

  /** Whether a note is detected (placeholder; wire to laser/beam sensors when available). */
  public boolean hasNote() {
    return false; // TODO: use LASER_1_CAN_ID / LASER_2_CAN_ID or DIO
  }

  public TalonFX getMotor() {
    return motor;
  }
}
