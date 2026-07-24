// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Value;

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
import frc.robot.constants.FieldConstants;
import frc.robot.Constants.Auto;
import frc.robot.commands.StrafeCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TurnToPointCommand;
import frc.robot.utils.NTChooser;

public class RobotContainer implements PowerRobotContainer {

  // --- Other ---
  private final StateMachine stateMachine = new StateMachine();
  private final NTChooser<AutoPath> autoChooser = new NTChooser<>("SmartDashboard/Auto Chooser");
  /**
   * Game timer: counts up from 0 to 2 minutes 40 seconds (160 s). Start via
   * {@link #startGameTimer()}.
   */

  public static final double GAME_DURATION_SECONDS = 2 * 60 + 40; // 2:40

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final VelocitySysId shooterSysId = new VelocitySysId(stateMachine.shooter, "Shooter");
  private final VelocitySysId feederSysId = new VelocitySysId(stateMachine.feeder, "Feeder");
  private final VelocitySysId spindexerSysId = new VelocitySysId(stateMachine.spindexer, "Spindexer");

  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Red Left", AutoPath.RED_LEFT);
    autoChooser.addOption("Red Right", AutoPath.RED_RIGHT);
    autoChooser.addOption("Blue Left", AutoPath.BLUE_LEFT);
    autoChooser.addOption("Blue Right", AutoPath.BLUE_RIGHT);

    // SysId: start log, run 4 tests per mechanism (quasistatic/dynamic, fwd/rev),
    // then stop log
    SmartDashboard.putData("SysId/Start Log", VelocitySysId.startLog());
    SmartDashboard.putData("SysId/Stop Log", VelocitySysId.stopLog());
    SmartDashboard.putData("SysId/Shooter Quasistatic Forward",
        shooterSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Shooter Quasistatic Reverse",
        shooterSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Shooter Dynamic Forward", shooterSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Shooter Dynamic Reverse", shooterSysId.dynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Feeder Quasistatic Forward",
        feederSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Feeder Quasistatic Reverse",
        feederSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Feeder Dynamic Forward", feederSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Feeder Dynamic Reverse", feederSysId.dynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Spindexer Quasistatic Forward",
        spindexerSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Spindexer Quasistatic Reverse",
        spindexerSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Spindexer Dynamic Forward", spindexerSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Spindexer Dynamic Reverse", spindexerSysId.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public void resetState() {
    stateMachine.setWantedState(RobotState.READY);
  }

  private void configureBindings() {
    // Intake in and out
    driverController.leftTrigger(0.5)
        // .or(driverController.leftTrigger(0.5))
        .onTrue(new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
              stateMachine.intakeRoller.setVelocity(145);
            }))
        .onFalse(new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
              stateMachine.intakeRoller.brake();
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
        }));

    driverController.b()
        .onTrue(new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
              stateMachine.intakeRoller.setVelocity(-100);
            }))
        .onFalse(new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
              stateMachine.intakeRoller.brake();
            }));

    // driverController.a().whileTrue(new StrafeCommand(stateMachine.drivetrain,
    // driverController));

    stateMachine.drivetrain.setDefaultCommand(new SwerveDriveCommand(stateMachine.drivetrain, driverController, false));

  }

  /** Echoes the resolved selection back to the dashboard. */
  public void updateAutoChooser() {
    autoChooser.publishActive();
  }

  public Command getAutonomousCommand() {
    AutoPath selected = autoChooser.get();

    if (selected == null) {
      return new InstantCommand(
          () -> System.out.println("No auto selected"));
    }

    switch (selected) {
      case RED_LEFT:
        return getRedLeftAuto();
      case RED_RIGHT:
        return getRedRightAuto();
      case BLUE_LEFT:
        return getBlueLeftAuto();
      case BLUE_RIGHT:
        return getBlueRightAuto();
      default:
        return new InstantCommand(
            () -> System.out.println("Invalid auto"));
    }
  }

  public static enum AutoPath {
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT
  }

  /** Builds the standard quadrant auto sequence with the given 7 poses. */
  private Command buildQuadrantAuto(
      Pose2d p1, Pose2d p2, Pose2d p3, Pose2d p4, Pose2d p5, Pose2d p6, Pose2d p7) {
    return new SequentialCommandGroup(
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p1, 6.0, 0.5),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p2, 6.0, 1.0, true),
        new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
              stateMachine.intakeRoller.setVelocity(145);
            }),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p3, 12.0, 0.75),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p4, 6.0, 0.75),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p5, 6.0, 0.4),
        new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
              stateMachine.intakeRoller.brake();
            }),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p6, 3.0, 0.75),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p7, 3.0, 1.0, true),
        new TurnToPointCommand(stateMachine.drivetrain,
            DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? FieldConstants.HOPPER_BLUE
                : FieldConstants.HOPPER_RED,
            5),
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
