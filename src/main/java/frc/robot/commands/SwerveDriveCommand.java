// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.DriveUtil;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.GameZone;
import frc.robot.constants.LocationConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDriveCommand extends Command {
  public double MAX_ANGULAR_RATE =
      RotationsPerSecond.of(1.5)
          .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE =
      RotationsPerSecond.of(0.5)
          .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double SKEW_COMPENSATION =
      0. - 0.03; // Adjust this value based on your robot's characteristics

  private final Swerve drivetrain;
  private final CommandXboxController controller;
  private final boolean autoDrive;
  private final PIDController driveToPointController;
  private Pose2d requestedPose;
  private double poseTolerance;

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = null;
    this.poseTolerance = -1;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = -1.0;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose,
      double poseTolerance) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = poseTolerance;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d currentPose = drivetrain.getState().Pose;
    final double speedCoefficient = FieldUtils.getZone(currentPose) == GameZone.INTERCHANGE ? OIConstants.INTERCHANGE_SPEED_COEFFICIENT : 1.0;
    Pose2d targetPose = new Pose2d();
    targetPose = requestedPose;

    if (currentPose != null && targetPose != null && (autoDrive || requestedPose != null)) {
      boolean isBlueAlliance = true;
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          isBlueAlliance = false;
        }
      }
      final double directionMultiplier = isBlueAlliance ? -1.0 : 1.0;

      Translation2d velocity = DriveUtil.calculateDriveToPointVelocity(
          currentPose, targetPose, directionMultiplier, driveToPointController, poseTolerance);

      drivetrain.drive(
          -velocity.getX() * speedCoefficient, -velocity.getY() * speedCoefficient, targetPose.getRotation().getDegrees(), Swerve.DriveMode.DRIVE_TO_POINT);
    // } else if (currentPose != null && targetPose != null && DriveUtil.isClose(currentPose, targetPose)) {
    //   final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(controller, drivetrain, MAX_ANGULAR_RATE, SKEW_COMPENSATION);
    //   drivetrain.drive(
    //       speeds.vxMetersPerSecond,
    //       speeds.vyMetersPerSecond,
    //       // currentPose.getRotation().getDegrees(),
    //       targetPose.getRotation().getDegrees(),
    //       Swerve.DriveMode.ROTATION_LOCK);
    } else if (controller.a().getAsBoolean()) {
      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(controller, drivetrain, MAX_ANGULAR_RATE, SKEW_COMPENSATION);
      StrafeSide  thatSide = getClosestSide(drivetrain);

      StrafeAxis axis = getStrafeAxis(thatSide);
      Pose2d pose = drivetrain.getState().Pose;
      GameZone zone = FieldUtils.getZone(pose);

      if (zone != GameZone.INTERCHANGE) {
        final double coeff = OIConstants.MAX_SPEED_COEFFICIENT;

        double speedX = axis == StrafeAxis.X ? speeds.vxMetersPerSecond * coeff : -getYInput(axis, thatSide);
        double speedY = axis == StrafeAxis.Y ? speeds.vyMetersPerSecond * coeff : -getXInput(axis, thatSide);

        drivetrain.drive(
            speedX,
            speedY,
            getRotation(axis != StrafeAxis.Y ? speeds.vxMetersPerSecond : speeds.vyMetersPerSecond, thatSide, zone, pose),
            Swerve.DriveMode.ROTATION_LOCK);
      } else {
        drivetrain.drive(
            speeds.vxMetersPerSecond * speedCoefficient,
            speeds.vyMetersPerSecond * speedCoefficient,
            -speeds.omegaRadiansPerSecond,
            Swerve.DriveMode.FIELD_RELATIVE);
      }
    } else {
      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(controller, drivetrain, MAX_ANGULAR_RATE, SKEW_COMPENSATION);
      final double coeff = speedCoefficient == 1.0 ? OIConstants.MAX_SPEED_COEFFICIENT : speedCoefficient;
      drivetrain.drive(
          speeds.vxMetersPerSecond * coeff,
          speeds.vyMetersPerSecond * coeff,
          -speeds.omegaRadiansPerSecond,
          Swerve.DriveMode.FIELD_RELATIVE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (requestedPose != null) {
      return getIsInPosition(
          drivetrain.getState().Pose, requestedPose, drivetrain.getState().Speeds);
    }
    return false;
  }


  private boolean getIsInPosition(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds speeds) {
    final Translation2d translationToPoint =
        currentPose.getTranslation().minus(targetPose.getTranslation());
    final double linearDistance = translationToPoint.getNorm();
    final double currentVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    final double tolerance = poseTolerance > 0 ? poseTolerance : 1;
    return linearDistance < Units.inchesToMeters(tolerance) && currentVelocity < 0.01; // meters
  }


  //
  //
  // Game Specific Code
  //
  //
  ///////////////////////////////////////////////////////////
  
  private double getRotation(double speed, StrafeSide side, GameZone zone, Pose2d pose) {
    Alliance alliance = DriverStation.getAlliance().get();
    double centerLine = getCenterLine(zone);

    switch (side) {
      case FRONT:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getY() > centerLine)
              || (alliance == Alliance.Red && pose.getY() < centerLine)) {
            return -45.0;
          } else {
            return 45.0;
          }
        } else if (speed > 0) {
          return 45.0;
        } else {
          return -45.0;
        }
      case LEFT:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getX() > 2 || (alliance == Alliance.Red && pose.getX() < 2))) {
            return 135.0;
          } else {
            return 45.0;
          }
        } else if (speed > 0) {
          return 45.0;
        } else {
          return 135.0;
        }
      case RIGHT:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getX() > 2) || (alliance == Alliance.Red && pose.getX() < 2)) {
            return -135.0;
          } else {
            return -45.0;
          }
        } else if (speed > 0.0) {
          return -45.0;
        } else {
          return -135.0;
        }
      case BACK:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getY() > centerLine)
              || (alliance == Alliance.Red && pose.getY() < centerLine)) {
            return 135.0;
          } else {
            return -135.0;
          }
        } else if (speed > 0) {
          return 135.0;
        } else {
          return -135.0;
        }
      default:
        return 0.0;
    }
  }

  private double getCenterLine(GameZone zone) {
    switch (zone) {
      case BLUE_ALLIANCE:
        return LocationConstants.BLUE_ZONE_X_MID;
      case RED_ALLIANCE:
        return LocationConstants.RED_ZONE_X_MID;
      case NEUTRAL:
        return LocationConstants.NEUTRAL_X_MID;
      default:
        return -1;
    }
  }

  private double getXInput(StrafeAxis axis, StrafeSide side) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().get();
    double strafeLine;

    if ((alliance == Alliance.Blue && side == StrafeSide.LEFT)
        ||
        (alliance == Alliance.Red && side == StrafeSide.RIGHT)) {
      strafeLine = LocationConstants.FAR_WALL_Y;
    } else {
      strafeLine = LocationConstants.NEAR_WALL_Y;
    }

    final Pose2d targetPose = new Pose2d(pose.getX(), strafeLine, pose.getRotation());
    final double directionMultiplier = alliance == Alliance.Blue ? -1.0 : 1.0;

    Translation2d velocity = DriveUtil.calculateDriveToPointVelocity(
        pose, targetPose, directionMultiplier, driveToPointController, poseTolerance);

    return velocity.getX();
  }

  private double getYInput(StrafeAxis axis, StrafeSide side) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().get();
    double strafeLine;

    switch (zone) {
      case BLUE_ALLIANCE:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.BLUE_HUB_WALL_X;
        } else {
          strafeLine = LocationConstants.BLUE_ALLIANCE_WALL_X;
        }

        break;
      case RED_ALLIANCE:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.RED_ALLIANCE_WALL_X;
        } else {
          strafeLine = LocationConstants.RED_HUB_WALL_X;
        }

      case NEUTRAL:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.RED_CENTER_WALL_X;
        } else {
          strafeLine = LocationConstants.BLUE_CENTER_WALL_X;
        }

      default:
        strafeLine = 0.0;
        break;
    }

    
    final Pose2d targetPose = new Pose2d(pose.getX(), strafeLine, pose.getRotation());
    final double directionMultiplier = alliance == Alliance.Blue ? -1.0 : 1.0;
    
    Translation2d velocity = DriveUtil.calculateDriveToPointVelocity(
      pose, targetPose, directionMultiplier, driveToPointController, poseTolerance);

    return velocity.getY();
  }

  /**
   * Returns the strafe side (wall) closest to the robot's current pose.
   * Uses alliance and zone to pick the correct wall X/Y constants.
   */
  private static StrafeSide getClosestSide(Swerve drivetrain) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    double frontWallX;
    double backWallX;
    switch (zone) {
      case BLUE_ALLIANCE:
        frontWallX = LocationConstants.BLUE_HUB_WALL_X;
        backWallX = LocationConstants.BLUE_ALLIANCE_WALL_X;
        break;
      case RED_ALLIANCE:
        frontWallX = LocationConstants.RED_HUB_WALL_X;
        backWallX = LocationConstants.RED_ALLIANCE_WALL_X;
        break;
      case NEUTRAL:
        frontWallX = alliance == Alliance.Blue ? LocationConstants.RED_CENTER_WALL_X
            : LocationConstants.BLUE_CENTER_WALL_X;
        backWallX = alliance == Alliance.Blue ? LocationConstants.BLUE_CENTER_WALL_X
            : LocationConstants.RED_CENTER_WALL_X;
        break;
      default:
        return StrafeSide.FRONT;
    }

    // Far/near wall Y are fixed field positions; LEFT/RIGHT map by alliance (see
    // getXInput).
    double distFront = Math.abs(pose.getX() - frontWallX);
    double distBack = Math.abs(pose.getX() - backWallX);
    double distToFarWall = Math.abs(pose.getY() - LocationConstants.FAR_WALL_Y);
    double distToNearWall = Math.abs(pose.getY() - LocationConstants.NEAR_WALL_Y);

    double min = Math.min(Math.min(distFront, distBack), Math.min(distToFarWall, distToNearWall));
    if (min == distFront)
      return StrafeSide.FRONT;
    if (min == distBack)
      return StrafeSide.BACK;
    if (min == distToFarWall)
      return alliance == Alliance.Blue ? StrafeSide.LEFT : StrafeSide.RIGHT;
    return alliance == Alliance.Blue ? StrafeSide.RIGHT : StrafeSide.LEFT;
  }

  private StrafeAxis getStrafeAxis(StrafeSide side) {
    if (side == StrafeSide.LEFT || side == StrafeSide.RIGHT) {
      return StrafeAxis.X;
    } else {
      return StrafeAxis.Y;
    }
  }

  private static enum StrafeAxis {
    X,
    Y
  }

  public static enum StrafeSide {
    LEFT,
    RIGHT,
    FRONT,
    BACK
  }

}