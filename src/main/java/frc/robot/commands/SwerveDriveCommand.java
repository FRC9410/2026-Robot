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

}