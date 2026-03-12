// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team9410.PowerRobotContainer;
import frc.lib.team9410.configs.SweepConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.AutoBuilder;
import frc.robot.utils.FieldUtils;
import frc.robot.commands.VelocitySysId;
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

  private final SendableChooser<SequentialCommandGroup> autoChooser = new AutoBuilder(stateMachine.drivetrain,
    driverController).build();

  private final VelocitySysId shooterSysId = new VelocitySysId(stateMachine.shooter, "Shooter");
  private final VelocitySysId feederSysId = new VelocitySysId(stateMachine.feeder, "Feeder");
  private final VelocitySysId spindexerSysId = new VelocitySysId(stateMachine.spindexer, "Spindexer");

  public RobotContainer() {
    configureBindings();

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

  public StateMachine getStateMachine() {
    return stateMachine;
  }
}
