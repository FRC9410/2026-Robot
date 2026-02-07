// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.*;
import java.util.List;

public class StateMachine extends SubsystemBase {
  Boolean isInPosition;
  Swerve drive;
  // Dashboard dashboard;

  public StateMachine(
      Swerve drive) {
    this.drive = drive;
    isInPosition = false;
  }

  RobotState INIT_STATE = RobotState.READY_STATE;
  RobotState READY_STATE = RobotState.READY_STATE;

  public static enum RobotState {
    READY_STATE, // This is the default state when the robot is not doing anything
    SCORING,
    PASSING
  }


  private RobotState wantedRobotState = INIT_STATE;
  private RobotState currentRobotState = INIT_STATE;
  private RobotState previousRobotState;

  @Override
  public void periodic() {
    handleRobotStateTransitions();
    
  }

  private void handleRobotStateTransitions() {
    boolean isBlueAlliance = true;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        isBlueAlliance = false;
      }
    }

    if (this.currentRobotState != this.wantedRobotState) {
      this.currentRobotState = READY_STATE;
      if (true) {
        this.currentRobotState = this.wantedRobotState;
      }
    }

    switch (this.currentRobotState) {
      case READY_STATE:
        executeReadyState();
        break;
    }
  }

  private boolean isReadyState() {
    if (true) { // check subsystems for readiness
      return true;
    }
    return false;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// State Execution Methods
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  private void executeReadyState() {
   
  }


  public void setWantedState(RobotState state) {
    wantedRobotState = state;
  }

  public RobotState getCurrentRobotState() {
    return currentRobotState;
  }
}
