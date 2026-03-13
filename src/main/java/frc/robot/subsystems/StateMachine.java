// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.FileSystem;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.FieldUtils.GameZone;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.TurretHelpers;

/** Subsystem that holds high-level robot state and drives transitions. */
public class StateMachine extends SubsystemBase {

  public enum RobotState {
    READY,
    SHOOTING
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

  private final Vision vision = new Vision();
  private final Dashboard dashboard = new Dashboard();

  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  private boolean winAuto = true;
  private boolean gyroReset = false;

  private int intakeTimer = 0;

  public StateMachine() {}

  @Override
  public void periodic() {
    if (currentState != RobotState.SHOOTING) {
      pointTurret(isBlueAlliance() ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED);
    }

    handleStateTransitions();
    executeState();
    PowerRobotContainer.setData("robotState", currentState.name());
    PowerRobotContainer.setData("currentZone", getZoneFromPRC());
    setRobotPose();

    
  
    // Map<String, Object> robotData = PowerRobotContainer.getAllData();
    // SmartDashboard.putData("robotData", builder -> {
    // builder.setSmartDashboardType("robotData");
    // for (String key : robotData.keySet()) {
    //     builder.addDoubleProperty(key, () -> robotData.get(key), v -> robotData.put(key, v));}});

  
    SmartDashboard.putString("gameZone", FieldUtils.getZone(drivetrain.getState().Pose).name());

  }

  public void resetGyro () {
    gyroReset = false;
  }

  public void setRobotPose () {
    String bestLimelight = vision.getBestLimelight();

    // Don't use vision when no Limelight has valid data (avoids bad/default pose from empty/wrong table on boot).
    if (bestLimelight == null || bestLimelight.isEmpty()) {
      // Skip vision pose update; dashboard/PRC still use current drivetrain pose below.
    } else {
      vision.setRobotPose(bestLimelight, TurretHelpers.getTurretAngle(turret.getPositionRotations() * 9/8.5));

      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimelight);
      if (mt2 != null && mt2.tagCount > 0 && mt2.avgTagArea > 0.1) {
        Pose2d newPose = mt2.pose;
        // Reject poses outside field bounds (e.g. 0,0 or garbage from cold Limelight).
        double x = newPose.getX();
        double y = newPose.getY();
        if (x < Constants.Field.X_MIN - Constants.Field.TOL || x > Constants.Field.X_MAX + Constants.Field.TOL
            || y < Constants.Field.Y_MIN - Constants.Field.TOL || y > Constants.Field.Y_MAX + Constants.Field.TOL) {
          // Bad pose; do not reset or add to estimator.
        } else {
          if (gyroReset) {
            newPose = new Pose2d(newPose.getX(), newPose.getY(), drivetrain.getState().Pose.getRotation());
          } else {
            gyroReset = true;
          }
          drivetrain.resetPose(newPose);
          drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
          drivetrain.addVisionMeasurement(newPose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
        }
      }
    }
      Translation2d translationToPoint = drivetrain.getState().Pose.getTranslation().minus(Constants.Field.HOPPER_RED);
      double linearDistance = translationToPoint.getNorm();

      PowerRobotContainer.setData("distanceToHopper", linearDistance);
      SmartDashboard.putNumber("distanceToHopper", linearDistance);
      SmartDashboard.putNumber("currentPoseX", drivetrain.getState().Pose.getX());
      SmartDashboard.putNumber("currentPoseY", drivetrain.getState().Pose.getY());
      SmartDashboard.putNumber("currentPoseYaw", drivetrain.getState().Pose.getRotation().getDegrees());
      SmartDashboard.putNumber("turretLimelightAngle", TurretHelpers.getTurretAngle(turret.getPositionRotations()));

      
      SmartDashboard.putNumber("turretOffsetX", TurretHelpers.turretCamPosRelative(TurretHelpers.getTurretAngle(turret.getPositionRotations())).getX());
      SmartDashboard.putNumber("turretOffsetY", TurretHelpers.turretCamPosRelative(TurretHelpers.getTurretAngle(turret.getPositionRotations())).getY());
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
      case SHOOTING:
        executeShooting();
        break;
      default:
        break;
    }
  }

  private void executeReady() {
    shooter.brake();
    spindexer.brake();
    feeder.brake();
    if (intakeWrist.getSetpointRotations() > Constants.Intake.INTAKE_IDLE){
      intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
    }
    intakeTimer = 0;
  }

  public GameZone getZoneFromPRC () {
    Pose2d pose = drivetrain.getState().Pose;

    return FieldUtils.getZone(pose);
  }

  private void executeShooting() {
    intakeTimer++;
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    boolean inOurZone = getAllianceZone() == zone;

    // if (inOurZone && !isHubActive()) {
    //   shooter.brake();
    //   feeder.brake();
    //   spindexer.brake();
    //   return;
    // }

    Translation2d target = inOurZone
        ? (zone == GameZone.BLUE_ALLIANCE ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED)
        : getTargetCornerLocation();
    runShootingToTarget(target);
    if (intakeWrist.getSetpointRotations() >= Constants.Intake.INTAKE_IDLE) {
      if (intakeTimer % 50 == 0) {
        intakeWrist.setPositionRotations(Constants.Intake.INTAKE_FEED);
      } else if (intakeTimer % 25 == 0) {
        intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
      }
    }
  }

  /** Runs shooter, hood, feeder, and spindexer toward the given target (hopper or corner). */
  private void runShootingToTarget(Translation2d target) {
    var state = drivetrain.getState();
    Pose2d pose = state.Pose;
    var speeds = state.Speeds;
    // double linearSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    // if (linearSpeedMps > TurretConstants.SHOOT_MAX_DRIVETRAIN_SPEED_MPS) {
    //   shooter.brake();
    //   feeder.brake();
    //   spindexer.brake();
    //   return;
    // }

    double turretRotation = TurretHelpers.getTurretRotationsWithoutLead(this, target) - pose.getRotation().getRadians();
    // if (Math.abs(turretRotation) > Math.PI / 2) {
    //   shooter.brake();
    //   feeder.brake();
    //   spindexer.brake();
    //   return;
    // }

    // Require turret to be within 2° of the angle to target before shooting
    // double targetTurretAngle = Math.atan2(
    //     target.getY() - pose.getY(),
    //     target.getX() - pose.getX());
    // double currentTurretAngle = turret.getPositionRotations() * 2 * Math.PI * (9.0 / 8.5);
    // if (Math.abs(targetTurretAngle - currentTurretAngle) > Math.toRadians(TurretConstants.TURRET_SHOOT_ANGLE_TOLERANCE_DEG)) {
    //   shooter.brake();
    //   feeder.brake();
    //   spindexer.brake();
    //   return;
    // }

    double normalizedDistance = pose.getTranslation().minus(target).getNorm();
    double rotationConstant = 0.0;
    double distance = normalizedDistance + Math.abs(turretRotation * rotationConstant);

    double shooterVelo = TurretConstants.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance);
    double hoodPos = TurretConstants.HOOD_ANGLE_INTERPOLATOR.getInterpolatedValue(distance);
    double feederVelo = TurretConstants.FEEDER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance);

    shooter.setVelocity(-shooterVelo - 1);
    shooterHood.setPositionRotations(hoodPos);
    feeder.setVelocity(-feederVelo);
    pointTurret(target);

    double velocityThreshold = TurretConstants.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance) - 2;
    System.out.println(spindexer.getVelocityMotor().getRotorVelocity());
    if (shooter.getVelocityMotor().getRotorVelocity().getValueAsDouble() < velocityThreshold) {
      if (intakeTimer > 25) {
        spindexer.setVelocity(60);
      } else {
        spindexer.setVelocity(95);
      }
    } else {
      feeder.brake();
      spindexer.brake();
    }
  }

  // private void pointTurret() {
  //   double turretRotation = TurretHelpers.getTurretRotationsWithoutLead(this) - drivetrain.getState().Pose.getRotation().getRadians();
  //   double rotationConstant = 0.09;

  //   double dir = turretRotation > 0 ? turretRotation + Math.abs(turretRotation * rotationConstant) : turretRotation - Math.abs(turretRotation * rotationConstant);
  //       if (Math.toDegrees(Math.abs(dir)) < 90) {
  //         turret.setPositionRotations(dir / Math.PI / 2 * (8.5/9));
  //         // System.out.println(dir / Math.PI * (8.5/9));
  //       }
  // }

  private void pointTurret(Translation2d point) {
    double turretRotation = TurretHelpers.getTurretRotationsWithoutLead(this, point) - drivetrain.getState().Pose.getRotation().getRadians();
    // System.out.println(turretRotation);
    // double rotationConstant = 0.09;

    // double dir = turretRotation > 0 ? turretRotation + Math.abs(turretRotation * rotationConstant) : turretRotation - Math.abs(turretRotation * rotationConstant);
    //     if (Math.toDegrees(Math.abs(dir)) < 90) {
    //       turret.setPositionRotations(dir / Math.PI / 2 * (8.5/9));
    //       // System.out.println(dir / Math.PI * (8.5/9));
    //     }

    double desiredTurretRotations = TurretHelpers.getTurretRotationsWithoutLead(this, point);

    // radians to turret setpoint
    double turretRotations = desiredTurretRotations / (2.0 * Math.PI);

    // restricting setpoint
    double clampedTurretRotations = MathUtil.clamp(turretRotations, -0.25, 0.25);

    // translating setpoint to gear ration
    double sensorRotations = clampedTurretRotations * (8.5 / 9.0);

    turret.setPositionRotations(sensorRotations);

    // delta 2pi and turret rotation
    // multiply delta * 8.5/9
    // apply new delta to turret rotation
    // continue...
  }

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
    if (DriverStation.getAlliance().isEmpty())
      return true;
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }

  /** Distance (m) inward from each side when targeting a corner (field: blue right = 0,0, red left = max, max). */
  private static final double CORNER_TARGET_OFFSET_M = 1.0;

  /**
   * Returns a target position 1 m inward from each side that forms the corner (end wall and side wall).
   * Blue right = (0,0), red left = (max, max). Picks the nearest of our alliance's two corners.
   */
  public Translation2d getTargetCornerLocation() {
    Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
    if (isBlueAlliance()) {
      // Blue corners: 1m from blue wall (low X) and 1m from bottom (low Y) or top (high Y)
      Translation2d fromBottom = new Translation2d(
          Constants.Field.BLUE_BOTTOM_CORNER.getX() + CORNER_TARGET_OFFSET_M,
          Constants.Field.BLUE_BOTTOM_CORNER.getY() + CORNER_TARGET_OFFSET_M);
      Translation2d fromTop = new Translation2d(
          Constants.Field.BLUE_TOP_CORNER.getX() + CORNER_TARGET_OFFSET_M,
          Constants.Field.BLUE_TOP_CORNER.getY() - CORNER_TARGET_OFFSET_M);
      return robotPos.getY() > 4 ? fromTop : fromBottom;
    } else {
      // Red corners: 1m from red wall (high X) and 1m from bottom (low Y) or top (high Y)
      Translation2d fromBottom = new Translation2d(
          Constants.Field.RED_BOTTOM_CORNER.getX() - CORNER_TARGET_OFFSET_M,
          Constants.Field.RED_BOTTOM_CORNER.getY() + CORNER_TARGET_OFFSET_M);
      Translation2d fromTop = new Translation2d(
          Constants.Field.RED_TOP_CORNER.getX() - CORNER_TARGET_OFFSET_M,
          Constants.Field.RED_TOP_CORNER.getY() - CORNER_TARGET_OFFSET_M);
      System.out.println(robotPos.getY() > 4 ? fromTop : fromBottom);
      return robotPos.getY() > 4 ? fromTop : fromBottom;
    }
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
