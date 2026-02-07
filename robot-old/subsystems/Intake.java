// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX rollerMotor;
  private final TalonFX primaryPivot;
  private final TalonFX secondaryPivot;
  private final CANcoder cancoder;

  private double upSetPoint;
  private double downSetPoint;
  private double voltage;
  /** Creates a new Intake. */
  public Intake() {
    rollerMotor =
        new TalonFX(Constants.IntakeConstants.ROLLER_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
    primaryPivot =
        new TalonFX(Constants.IntakeConstants.PRIMARY_PIVOT_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
    secondaryPivot =
        new TalonFX(Constants.IntakeConstants.SECONDARY_PIVOT_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
    cancoder =
        new CANcoder(Constants.IntakeConstants.ENCODER_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);

    rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    primaryPivot.setNeutralMode(NeutralModeValue.Brake);
    secondaryPivot.setNeutralMode(NeutralModeValue.Brake);
    
    upSetPoint = Constants.IntakeConstants.INTAKE_UP_SETPOINT;
    downSetPoint = Constants.IntakeConstants.INTAKE_DOWN_SETPOINT;
    voltage = Constants.IntakeConstants.INTAKE_ROLLER_VOLTAGE;

    primaryPivot.setControl(new Follower(primaryPivot.getDeviceID(), MotorAlignmentValue.Opposed));


    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
