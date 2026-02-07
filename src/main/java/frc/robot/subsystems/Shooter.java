// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for shooter flywheels (primary + secondary). */
public class Shooter extends SubsystemBase {

  private final TalonFX primaryMotor;
  private final TalonFX secondaryMotor;

  public Shooter() {
    primaryMotor =
        new TalonFX(
            Constants.ShooterConstants.PRIMARY_FLYWHEELS_CAN_ID,
            Constants.CanBusConstants.CANIVORE_BUS);
    secondaryMotor =
        new TalonFX(
            Constants.ShooterConstants.SECONDARY_FLYWHEELS_CAN_ID,
            Constants.CanBusConstants.CANIVORE_BUS);

    primaryMotor.setNeutralMode(NeutralModeValue.Coast);
    secondaryMotor.setNeutralMode(NeutralModeValue.Coast);
    // secondaryMotor.setControl(new Follower(Constants.ShooterConstants.PRIMARY_FLYWHEELS_CAN_ID, true));
  }

  @Override
  public void periodic() {}

  /** Set flywheel output (applied to primary; secondary follows). */
  public void setOutput(double output) {
    primaryMotor.set(output);
  }

  /** Set flywheel velocity in rotations per second (placeholder; use closed-loop when tuned). */
  public void setVelocityRPS(double rps) {
    primaryMotor.set(rps / 50.0); // placeholder open-loop; replace with VelocityVoltage
  }

  /** Stop the flywheels. */
  public void stop() {
    primaryMotor.set(0);
  }

  /** Get primary flywheel velocity in rotations per second. */
  // public double getVelocityRPS() {
  //   return primaryMotor.getVelocity().getValue();
  // }

  public TalonFX getPrimaryMotor() {
    return primaryMotor;
  }

  public TalonFX getSecondaryMotor() {
    return secondaryMotor;
  }
}
