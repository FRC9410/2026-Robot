// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LocationConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.DriveUtil;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.GameZone;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StrafeCommand extends Command {
  public double MAX_ANGULAR_RATE =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE =
      RotationsPerSecond.of(0.5)
          .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit

  private final Swerve drivetrain;
  private final CommandXboxController controller;
  private final StrafeSide side;
  private final PIDController driveToPointController;
  private final double poseTolerance;
  
    public StrafeCommand(
        Swerve drivetrain,
        CommandXboxController controller,
        StrafeSide side) {
      this.drivetrain = drivetrain;
      this.controller = controller;
      this.side = side;
      this.driveToPointController = new PIDController(3.2, 0, 0.2);
      this.poseTolerance = -1;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    StrafeAxis axis = getStrafeAxis(side);
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    if (zone != GameZone.INTERCHANGE) {
      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(controller, drivetrain, MAX_ANGULAR_RATE, 0.0);

      drivetrain.drive(
        axis == StrafeAxis.Y ? speeds.vxMetersPerSecond : getXInput(axis, side),
        axis == StrafeAxis.X ? speeds.vyMetersPerSecond : getYInput(axis, side),
        getRotation(axis == StrafeAxis.Y ? speeds.vxMetersPerSecond : speeds.vyMetersPerSecond, side, zone, pose),
        Swerve.DriveMode.ROTATION_LOCK
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getRotation (double speed, StrafeSide side, GameZone zone, Pose2d pose) {
    Alliance alliance = DriverStation.getAlliance().get();
    double centerLine = getCenterLine(zone);

    switch (side) {
      case FRONT:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getY() > centerLine) || (alliance == Alliance.Red && pose.getY() < centerLine)) {
            return 45.0;
          } else {
            return -45.0;
          }
        } else if (speed > 0) {
          return -45.0;
        } else {
          return 45.0;
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
          if ((alliance == Alliance.Blue && pose.getY() > centerLine) || (alliance == Alliance.Red && pose.getY() < centerLine)) {
            return -135.0;
          } else {
            return 135.0;
          }
        } else if (speed > 0) {
          return -135.0;
        } else {
          return 135.0;
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
        return LocationConstants.NO_MANS_LAND_X_MID;
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
