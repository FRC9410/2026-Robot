// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;

/** Subsystem that holds high-level robot state and drives transitions. */
public class StateMachine extends SubsystemBase {

  public enum RobotState {
    READY,
    INTAKING,
    AIMING,
    SHOOTING,
    PASSING,
    CLIMBING
  }

  private RobotState wantedState = RobotState.READY;
  private RobotState currentState = RobotState.READY;
  private RobotState previousState = RobotState.READY;

  public StateMachine() {}

  @Override
  public void periodic() {
    handleStateTransitions();
    executeState();
    PowerRobotContainer.setData("robotState", currentState.name());
  }

  private void handleStateTransitions() {
    if (currentState != wantedState) {
      previousState = currentState;
      if (canTransitionTo(wantedState)) {
        currentState = wantedState;
      }
    }
  }

  /** Override to add transition guards (e.g. require subsystems ready). */
  private boolean canTransitionTo(RobotState target) {
    return true;
  }

  private void executeState() {
    switch (currentState) {
      case READY:
        executeReady();
        break;
      case INTAKING:
        executeIntaking();
        break;
      case AIMING:
        executeAiming();
        break;
      case SHOOTING:
        executeShooting();
        break;
      case PASSING:
        executePassing();
        break;
      case CLIMBING:
        executeClimbing();
        break;
      default:
        break;
    }
  }

  private void executeReady() {}

  private void executeIntaking() {}

  private void executeAiming() {}

  private void executeShooting() {}

  private void executePassing() {}

  private void executeClimbing() {} // we arent doing this im pretty sure

  public void setWantedState(RobotState state) {
    wantedState = state;
  }

  public RobotState getCurrentState() {
    return currentState;
  }

  public RobotState getWantedState() {
    return wantedState;
  }

  public RobotState getPreviousState() {
    return previousState;
  }

  /** Convenience: is the robot on the blue alliance? */
  public boolean isBlueAlliance() {
    if (DriverStation.getAlliance().isEmpty()) return true;
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }
}
