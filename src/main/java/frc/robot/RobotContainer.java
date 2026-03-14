// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team9410.PowerRobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.VelocitySysId;
import frc.robot.constants.AutoConstants;
import frc.robot.commands.StrafeCommand;
import frc.robot.commands.SwerveDriveCommand;

public class RobotContainer implements PowerRobotContainer {

  // --- Other ---
  private final StateMachine stateMachine = new StateMachine();
  /**
   * Game timer: counts up from 0 to 2 minutes 40 seconds (160 s). Start via
   * {@link #startGameTimer()}.
   */
  
  public static final double GAME_DURATION_SECONDS = 2 * 60 + 40; // 2:40

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final VelocitySysId shooterSysId = new VelocitySysId(stateMachine.shooter, "Shooter");
  private final VelocitySysId feederSysId = new VelocitySysId(stateMachine.feeder, "Feeder");
  private final VelocitySysId spindexerSysId = new VelocitySysId(stateMachine.spindexer, "Spindexer");

  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("Red Left", getRedLeftAuto());
    autoChooser.addOption("Red Right", getRedRightAuto());
    autoChooser.addOption("Blue Left", getBlueLeftAuto());
    autoChooser.addOption("Blue Right", getBlueRightAuto());
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // SysId: start log, run 4 tests per mechanism (quasistatic/dynamic, fwd/rev), then stop log
    SmartDashboard.putData("SysId/Start Log", VelocitySysId.startLog());
    SmartDashboard.putData("SysId/Stop Log", VelocitySysId.stopLog());
    SmartDashboard.putData("SysId/Shooter Quasistatic Forward", shooterSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Shooter Quasistatic Reverse", shooterSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Shooter Dynamic Forward", shooterSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Shooter Dynamic Reverse", shooterSysId.dynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Feeder Quasistatic Forward", feederSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Feeder Quasistatic Reverse", feederSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Feeder Dynamic Forward", feederSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Feeder Dynamic Reverse", feederSysId.dynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Spindexer Quasistatic Forward", spindexerSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Spindexer Quasistatic Reverse", spindexerSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Spindexer Dynamic Forward", spindexerSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Spindexer Dynamic Reverse", spindexerSysId.dynamic(SysIdRoutine.Direction.kReverse));
  }

  private void configureBindings() {
    // Intake in and out
    driverController.leftTrigger(0.5)
     //   .or(driverController.leftTrigger(0.5))
        .onTrue(new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
              stateMachine.intakeRoller.setVelocity(125);
            }))
        .onFalse(new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
              stateMachine.intakeRoller.brake();
            }));
    // Intake in and out
    driverController.y()
     //   .or(driverController.leftTrigger(0.5))
        .onTrue(new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_FEED);
            }))
        .onFalse(new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
            }));
    
    driverController.rightTrigger(0.5).onTrue(new InstantCommand(
        () -> {
          stateMachine.setWantedState(RobotState.SHOOTING);
        })).onFalse(new InstantCommand(
        () -> {
          stateMachine.setWantedState(RobotState.READY);
        }));

    driverController.back().onTrue(new InstantCommand(
      () -> {
        stateMachine.resetGyro();
      }
    ));

    driverController.a().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController));

    stateMachine.drivetrain.setDefaultCommand(new SwerveDriveCommand(stateMachine.drivetrain, driverController, false));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Builds the standard quadrant auto sequence with the given 7 poses. */
  private Command buildQuadrantAuto(
      Pose2d p1, Pose2d p2, Pose2d p3, Pose2d p4, Pose2d p5, Pose2d p6, Pose2d p7) {
    return new SequentialCommandGroup(
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p1, 6.0),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p2, 12.0),
        new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
              stateMachine.intakeRoller.setVelocity(125);
            }),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p3, 12.0),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p4, 6.0),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p5, 6.0, 0.5),
        new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
              stateMachine.intakeRoller.brake();
            }),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p6, 3.0),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p7, 3.0),
        new InstantCommand(() -> stateMachine.setWantedState(RobotState.SHOOTING)));
  }

  public Command getRedLeftAuto() {
    return buildQuadrantAuto(
        AutoConstants.RED_LEFT_1, AutoConstants.RED_LEFT_2, AutoConstants.RED_LEFT_3,
        AutoConstants.RED_LEFT_4, AutoConstants.RED_LEFT_5, AutoConstants.RED_LEFT_6, AutoConstants.RED_LEFT_7);
  }

  public Command getRedRightAuto() {
    return buildQuadrantAuto(
        AutoConstants.RED_RIGHT_1, AutoConstants.RED_RIGHT_2, AutoConstants.RED_RIGHT_3,
        AutoConstants.RED_RIGHT_4, AutoConstants.RED_RIGHT_5, AutoConstants.RED_RIGHT_6, AutoConstants.RED_RIGHT_7);
  }

  public Command getBlueLeftAuto() {
    return buildQuadrantAuto(
        AutoConstants.BLUE_LEFT_1, AutoConstants.BLUE_LEFT_2, AutoConstants.BLUE_LEFT_3,
        AutoConstants.BLUE_LEFT_4, AutoConstants.BLUE_LEFT_5, AutoConstants.BLUE_LEFT_6, AutoConstants.BLUE_LEFT_7);
  }

  public Command getBlueRightAuto() {
    return buildQuadrantAuto(
        AutoConstants.BLUE_RIGHT_1, AutoConstants.BLUE_RIGHT_2, AutoConstants.BLUE_RIGHT_3,
        AutoConstants.BLUE_RIGHT_4, AutoConstants.BLUE_RIGHT_5, AutoConstants.BLUE_RIGHT_6, AutoConstants.BLUE_RIGHT_7);
  }

  public StateMachine getStateMachine() {
    return stateMachine;
  }
}
