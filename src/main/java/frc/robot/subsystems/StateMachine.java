// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;
import frc.lib.team9410.subsystems.PositionSubsystem;
import frc.lib.team9410.subsystems.VelocitySubsystem;
import frc.robot.Constants;

/** Subsystem that holds high-level robot state and drives transitions. */
public class StateMachine extends SubsystemBase {

  public enum RobotState {
    READY,
    SCORING,
    PASSING,
    CLIMBING
  }

  private RobotState wantedState = RobotState.READY;
  private RobotState currentState = RobotState.READY;
  private RobotState previousState = RobotState.READY;

  // --- Position subsystems ---
  public final PositionSubsystem turret = new PositionSubsystem(Constants.Turret.TURRET_CONFIG);
  public final PositionSubsystem shooterHood = new PositionSubsystem(Constants.Shooter.HOOD_CONFIG);
  public final PositionSubsystem intakeWrist = new PositionSubsystem(Constants.Intake.WRIST_CONFIG);

  // --- Velocity subsystems ---
  public final VelocitySubsystem shooter = new VelocitySubsystem(Constants.Shooter.FLYWHEEL_CONFIG);
  public final VelocitySubsystem intakeRoller = new VelocitySubsystem(Constants.Intake.ROLLER_CONFIG);
  public final VelocitySubsystem spindexer = new VelocitySubsystem(Constants.Spindexer.SPINDEXER_CONFIG);
  public final VelocitySubsystem feeder = new VelocitySubsystem(Constants.Feeder.FEEDER_CONFIG);

  // other
  private final LED led = new LED();
  private final Vision vision = new Vision();
  private final Dashboard dashboard = new Dashboard();

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
      case SCORING:
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

  private void executeShooting() {
    Pose2d pose = PowerRobotContainer.getData("robotPose", new Pose2d());
    if (PowerRobotContainer.getData("robotPose") == null) {
      return; // pose hasnt been updated yet
    }

    GameZone zone = getZone(pose);

    if (getAllianceZone() == zone) { // we are in our zone
      // if ()
    } else { // we are not in our zone
      
    }
  }

  private void executePassing() {

  }

  private void executeClimbing() {

  } // we arent doing this im pretty sure

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
    return DriverStation.getAlliance().get
    
    () == Alliance.Blue;
  }

  public GameZone getAllianceZone () {
    if (isBlueAlliance()) {
      return GameZone.BLUE_ALLIANCE;
    } else {
      return GameZone.RED_ALLIANCE;
    }
  }

  public enum GameZone {
    RED_ALLIANCE, // max, max
    NEUTRAL,
    BLUE_ALLIANCE, // 0,0
    INTERCHANGE // not in any specific zone, between alliance and neutral aka going over bump
  }

  public GameZone getZone (Pose2d pose) {
    double x = pose.getX();

    if (Constants.Field.BLUE_START_X < x && Constants.Field.BLUE_END_X > x) {
      return GameZone.BLUE_ALLIANCE;
    }

    if (Constants.Field.CENTER_START_X < x && Constants.Field.CENTER_END_X > x) {
      return GameZone.NEUTRAL;
    }

    if (Constants.Field.RED_START_X < x && Constants.Field.RED_END_X > x) {
      return GameZone.RED_ALLIANCE;
    }

    return GameZone.INTERCHANGE;
  }
}
