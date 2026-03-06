// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team9410.PowerRobotContainer;
import frc.lib.team9410.configs.SweepConfig;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.utils.CommandBuilder;
import frc.robot.utils.SweepHelpers;
import frc.robot.utils.SweepHelpers.ControllerButton;
import frc.robot.utils.SweepHelpers.SweepDirection;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.AutoBuilder;
import frc.robot.commands.VelocitySysId;
import frc.robot.commands.StrafeCommand;
import frc.robot.commands.StrafeCommand.StrafeSide;
import frc.robot.commands.SwerveDriveCommand;

public class RobotContainer implements PowerRobotContainer {

  // --- Other ---
  private final StateMachine stateMachine = new StateMachine();

  private boolean spindexerOn = false;
  private boolean shooterOn = false;
  private boolean feederOn = false;
  private boolean intakeOn = false;
  private boolean shooterHoodOn = false;
  private boolean turretOn = false;

  /**
   * Game timer: counts up from 0 to 2 minutes 40 seconds (160 s). Start via
   * {@link #startGameTimer()}.
   */
  
  public static final double GAME_DURATION_SECONDS = 2 * 60 + 40; // 2:40

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController testController = new CommandXboxController(5);
  private final SweepConfig sweepConfig = new SweepConfig(stateMachine.drivetrain, driverController, stateMachine);

  private final SendableChooser<SequentialCommandGroup> autoChooser = new AutoBuilder(stateMachine.drivetrain,
      testController, stateMachine).build();

  private final VelocitySysId shooterSysId = new VelocitySysId(stateMachine.shooter, "Shooter");
  private final VelocitySysId feederSysId = new VelocitySysId(stateMachine.feeder, "Feeder");
  private final VelocitySysId spindexerSysId = new VelocitySysId(stateMachine.spindexer, "Spindexer");

  public RobotContainer() {
    configureBindings();
    configureTestBindings();

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
    driverController.rightTrigger(0.5)
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

    driverController.leftTrigger(0.5).onTrue(
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new InstantCommand(
              () -> {
                double hoodTarget = PowerRobotContainer.getData("Shooter HoodTarget", Constants.Shooter.SHOOTER_HOOD_DEFAULT);
                stateMachine.shooterHood.setPositionRotations(hoodTarget);
              },
              stateMachine.shooterHood),
          new WaitUntilCommand(() -> {
            double hoodTarget = PowerRobotContainer.getData("Shooter HoodTarget", Constants.Shooter.SHOOTER_HOOD_DEFAULT);
            double actual = stateMachine.shooterHood.getPositionRotations();
            return Math.abs(actual - hoodTarget) < 0.005;
          })),
        new ParallelRaceGroup(
          new InstantCommand(
              () -> {
                double shooterVel = PowerRobotContainer.getData("ShooterVelocity", 100.0);
                stateMachine.shooter.setVelocity(-shooterVel);
              },
              stateMachine.shooter),
          new WaitUntilCommand(() -> {
            double shooterTarget = PowerRobotContainer.getData("ShooterVelocity", 100.0);
            double actual = Math.abs(stateMachine.shooter.getVelocityMotor().getRotorVelocity().getValueAsDouble());
            return Math.abs(actual - shooterTarget) < 5;
          })),
        new ParallelRaceGroup(
          new InstantCommand(
              () -> {
                double feederVel = PowerRobotContainer.getData("FeederVelocity", 95.0);
                stateMachine.feeder.setVelocity(-feederVel);
              },
              stateMachine.feeder),
          new WaitUntilCommand(() -> {
            double feederTarget = PowerRobotContainer.getData("FeederVelocity", 95.0);
            double actual = Math.abs(stateMachine.feeder.getVelocityMotor().getRotorVelocity().getValueAsDouble());
            return Math.abs(actual - feederTarget) < 5;
          })),
        new InstantCommand(
            () -> {
              double spindexerVel = PowerRobotContainer.getData("SpindexerVelocity", 150.0);
              stateMachine.spindexer.setVelocity(spindexerVel);
            },
            stateMachine.spindexer)
      )).onFalse(new InstantCommand(
      () -> {
          stateMachine.shooter.brake();
          stateMachine.feeder.brake();
          stateMachine.spindexer.brake();
      }, stateMachine.shooter, stateMachine.feeder, stateMachine.spindexer
    ));

    // State changers
    driverController.leftBumper().onTrue(new InstantCommand(
        () -> {
          stateMachine.setWantedState(RobotState.PASSING);
        }));
    driverController.rightBumper().onTrue(new InstantCommand(
        () -> {
          stateMachine.setWantedState(RobotState.SCORING);
        }));

    // // Sweeping commands
    // driverController.a().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController, StrafeSide.BACK));
    
    // driverController.b().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController, StrafeSide.RIGHT));
    
    // driverController.x().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController, StrafeSide.LEFT));
    
    // driverController.y().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController, StrafeSide.FRONT));

    stateMachine.drivetrain.setDefaultCommand(new SwerveDriveCommand(stateMachine.drivetrain, driverController, stateMachine, false));

  }
  // Test bindings
  private void configureTestBindings() {
    testController.a().onTrue(new InstantCommand(
        () -> {
          if (spindexerOn) {
            stateMachine.spindexer.brake();
          } else {
            stateMachine.spindexer.setVelocity(175);
          }

          spindexerOn = !spindexerOn;
        }));
    testController.b().onTrue(new InstantCommand(
        () -> {
          if (feederOn) {
            stateMachine.feeder.brake();
          } else {
            stateMachine.feeder.setVelocity(-200);
          }

          feederOn = !feederOn;
        }));
    // testController.leftBumper().onTrue(new InstantCommand(
    //     () -> {
    //       if (turretOn) {
    //         stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
    //       } else {
    //         stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
    //       }

    //       turretOn = !turretOn;
    //     }));

    testController.y().onTrue(new InstantCommand(
        () -> {
          if (shooterOn) {
            stateMachine.shooter.brake();
          } else {
            stateMachine.shooter.setVelocity(-100);
          }

          shooterOn = !shooterOn;
        }));
    testController.x().onTrue(new InstantCommand(
        () -> {
          if (intakeOn) {
            stateMachine.intakeRoller.brake();
            stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
          } else {
            stateMachine.intakeRoller.setVelocity(100); // 200 works if needed or wanted
            stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
          }

          intakeOn = !intakeOn;
        }));
    testController.rightBumper().onTrue(new InstantCommand(
        () -> {
          if (shooterHoodOn) {
            stateMachine.shooterHood.setPositionRotations(0.05);
          } else {
            stateMachine.shooterHood.setPositionRotations(Constants.Shooter.SHOOTER_HOOD_MIN);
          }

          shooterHoodOn = !shooterHoodOn;
        }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public StateMachine getStateMachine() {
    return stateMachine;
  }
}
