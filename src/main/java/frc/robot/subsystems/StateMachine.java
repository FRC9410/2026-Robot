// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;
import frc.lib.team9410.subsystems.PositionSubsystem;
import frc.lib.team9410.subsystems.VelocitySubsystem;
import frc.robot.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.utils.FieldUtils.GameZone;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.TurretHelpers;

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

  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  private boolean winAuto = true;

  public StateMachine() {}

  @Override
  public void periodic() {
    handleStateTransitions();
    executeState();
    PowerRobotContainer.setData("robotState", currentState.name());
    PowerRobotContainer.setData("currentZone", getZoneFromPRC());

    vision.setRobotPose();


    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-turret");
    if (mt2 != null && mt2.tagCount > 0) {
      drivetrain.resetPose(mt2.pose);
      drivetrain.resetRotation(mt2.pose.getRotation());
      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      drivetrain.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));

      Translation2d translationToPoint = mt2.pose.getTranslation().minus(Constants.Field.HOPPER_RED);
      double linearDistance = translationToPoint.getNorm();
      PowerRobotContainer.setData("distanceToHopper", linearDistance);

      SmartDashboard.putNumber("distanceToHopper", linearDistance);
    }
    


    // Map<String, Object> robotData = PowerRobotContainer.getAllData();
    // SmartDashboard.putData("robotData", builder -> {
    // builder.setSmartDashboardType("robotData");
    // for (String key : robotData.keySet()) {
    //     builder.addDoubleProperty(key, () -> robotData.get(key), v -> robotData.put(key, v));}});

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

  private void executeReady() {
  }

  public GameZone getZoneFromPRC () {
    Pose2d pose = PowerRobotContainer.getData("robotPose", new Pose2d());
    if (PowerRobotContainer.getData("robotPose") == null) {
      return GameZone.INTERCHANGE; // pose hasnt been updated yet
    }

    return FieldUtils.getZone(pose);
  }

  private void executeShooting() {
    Pose2d pose = PowerRobotContainer.getData("robotPose", new Pose2d());
    if (PowerRobotContainer.getData("robotPose") == null) {
      return; // pose hasnt been updated yet
    }

    GameZone zone = FieldUtils.getZone(pose);

    if (getAllianceZone() == zone) { // we are in our zone
      if (isHubActive()) {
        // remember to check for balls
        
        if (!shooter.isAllMotorsRunning()) {
          shooter.setVelocity(1); //placeholder
        }
      } else {
        if (shooter.isAllMotorsRunning()) {
          shooter.stopAll();
        }
      }
    } else { // we are not in our zone
      if (shooter.isAllMotorsRunning()) {
        shooter.stopAll(); // stop shooter flywheels
      }
    }
  }

  private void executePassing() {
    Pose2d pose = PowerRobotContainer.getData("robotPose", new Pose2d());
    if (PowerRobotContainer.getData("robotPose") == null) {
      return; // pose hasnt been updated yet
    }

    GameZone zone = FieldUtils.getZone(pose);

    if (getAllianceZone() == zone) { // we are in our zone
      if (shooter.isAllMotorsRunning()) {
        shooter.stopAll();
      }
    } else { // we are not in our zone
      if (!shooter.isAllMotorsRunning()) {
        shooter.setVelocity(1); // placeholder i think
      }

      if (pose.getY() > 4.0) { // we are on top half of field
        if (isBlueAlliance()) {
          TurretHelpers.setTarget(Constants.Field.BLUE_TOP_CORNER);
        } else {
          TurretHelpers.setTarget(Constants.Field.BLUE_BOTTOM_CORNER);
        }
      } else { // bottom half of the field
        if (isBlueAlliance()) {
          TurretHelpers.setTarget(Constants.Field.RED_TOP_CORNER);
        } else {
          TurretHelpers.setTarget(Constants.Field.RED_BOTTOM_CORNER);
        }
      }
    }
  }

  private void executeClimbing() {

  } // we arent doing this im pretty sure

  public void setWantedState(RobotState state) {
    wantedState = state;
  }

  private void aimer() {
    TurretHelpers.calculateHoodAngle(0);
    TurretHelpers.calculateShooterVelocity(0);
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
    if (DriverStation.getAlliance().isEmpty())
      return true;
    return DriverStation.getAlliance().get

    () == Alliance.Blue;
  }

  public GameZone getAllianceZone() {
    if (isBlueAlliance()) {
      return GameZone.BLUE_ALLIANCE;
    } else {
      return GameZone.RED_ALLIANCE;
    }
  }


  // thanks wpilib
  // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
}
