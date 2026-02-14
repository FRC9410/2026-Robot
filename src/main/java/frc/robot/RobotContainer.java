// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team9410.PowerRobotContainer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.lib.team9410.subsystems.PositionSubsystem;
import frc.lib.team9410.subsystems.VelocitySubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer implements PowerRobotContainer {

  // --- Position subsystems ---
  private final PositionSubsystem turret = new PositionSubsystem(
      Constants.TurretConstants.TURRET_MOTOR_CONFIGS,
      Constants.TurretConstants.TURRET_LEAD_CONFIG,
      Constants.TurretConstants.TURRET_CANCODER_CONFIG,
      Constants.TurretConstants.TURRET_MOTION_MAGIC_CONFIG,
      "Turret",
      "degrees");
  private final PositionSubsystem shooterHood = new PositionSubsystem(
      Constants.ShooterConstants.HOOD_MOTOR_CONFIGS,
      Constants.ShooterConstants.HOOD_LEAD_CONFIG,
      Constants.ShooterConstants.HOOD_CANCODER_CONFIG,
      Constants.ShooterConstants.HOOD_MOTION_MAGIC_CONFIG,
      "Shooter Hood",
      "degrees");
  private final PositionSubsystem intakeWrist = new PositionSubsystem(
      Constants.IntakeConstants.WRIST_MOTOR_CONFIGS,
      Constants.IntakeConstants.WRIST_LEAD_CONFIG,
      Constants.IntakeConstants.WRIST_CANCODER_CONFIG,
      Constants.IntakeConstants.WRIST_MOTION_MAGIC_CONFIG,
      "Intake Wrist",
      "degrees");

  // --- Velocity subsystems ---
  private final VelocitySubsystem shooter = new VelocitySubsystem(
      Constants.ShooterConstants.FLYWHEEL_MOTOR_CONFIGS,
      Constants.ShooterConstants.FLYWHEEL_LEAD_CONFIG,
      Constants.ShooterConstants.FLYWHEEL_MOTION_MAGIC_CONFIG,
      "Shooter");
  private final VelocitySubsystem intakeRoller = new VelocitySubsystem(
      Constants.IntakeConstants.ROLLER_MOTOR_CONFIGS,
      Constants.IntakeConstants.ROLLER_LEAD_CONFIG,
      Constants.IntakeConstants.ROLLER_MOTION_MAGIC_CONFIG,
      "Intake Roller");
  private final VelocitySubsystem spindexer = new VelocitySubsystem(
      Constants.SpindexerConstants.SPINDEXER_MOTOR_CONFIGS,
      Constants.SpindexerConstants.SPINDEXER_LEAD_CONFIG,
      Constants.SpindexerConstants.SPINDEXER_MOTION_MAGIC_CONFIG,
      "Spindexer");
  private final VelocitySubsystem feeder = new VelocitySubsystem(
      Constants.FeederConstants.MOTOR_CONFIGS,
      Constants.FeederConstants.FEEDER_LEAD_CONFIG,
      Constants.FeederConstants.FEEDER_MOTION_MAGIC_CONFIG,
      "Feeder");

  // --- Other ---
  private final LED led = new LED();
  private final Vision vision = new Vision();
  private final StateMachine stateMachine = new StateMachine();
  private final Dashboard dashboard = new Dashboard();

  /** Game timer: counts up from 0 to 2 minutes 40 seconds (160 s). Start via {@link #startGameTimer()}. */
  public static final double GAME_DURATION_SECONDS = 2 * 60 + 40; // 2:40
  private final Timer gameTimer = new Timer();

  // Controller
  private final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandXboxController TestController = new CommandXboxController(5);

  public RobotContainer() {
    configureBindings();
    configureTestBindings();

    PowerRobotContainer.setData("spindexerVelocity", 24);
    PowerRobotContainer.setData("feederVelocity", 24);
    PowerRobotContainer.setData("shooterVelocity", 24);
  }

  private void configureBindings() {


  }

  private void runSubsystem (VelocitySubsystem subsystem) {
    // System.out.println(subsystem.getClass().getName());
    subsystem.setVelocity(24);
  }

  private void configureTestBindings() {
    TestController.a().toggleOnTrue(new InstantCommand(
      () -> runSubsystem(spindexer)
    ));
    TestController.b().toggleOnTrue(new InstantCommand(
      () -> runSubsystem(feeder)
    ));
    TestController.y().toggleOnTrue(new InstantCommand(
      () -> runSubsystem(shooter)
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public LED getLED() {
    return led;
  }

  public PositionSubsystem getTurret() {
    return turret;
  }

  public VelocitySubsystem getShooter() {
    return shooter;
  }

  public PositionSubsystem getShooterHood() {
    return shooterHood;
  }

  public Vision getVision() {
    return vision;
  }

  public VelocitySubsystem getIntakeRoller() {
    return intakeRoller;
  }

  public PositionSubsystem getIntakeWrist() {
    return intakeWrist;
  }

  public VelocitySubsystem getSpindexer() {
    return spindexer;
  }

  public VelocitySubsystem getFeeder() {
    return feeder;
  }

  public StateMachine getStateMachine() {
    return stateMachine;
  }

  /** Call when the match starts (e.g. from Robot.java on first enable) to start the game timer. */
  public void startGameTimer() {
    if (!gameTimer.isRunning()) {
      gameTimer.reset(); // start from 0
      gameTimer.start();
    }
  }

  /** Same as {@link #getGameTime()} — elapsed seconds counting up from 0. */
  public double getGameTimeElapsed() {
    return gameTimer.get();
  }

  /** Remaining game time in seconds (0 when elapsed >= 2:40). */
  public double getGameTimeRemaining() {
    return Math.max(0, GAME_DURATION_SECONDS - gameTimer.get());
  }

  /** True when elapsed time has reached or passed 2 minutes 40 seconds. */
  public boolean isGameTimeUp() {
    return gameTimer.get() >= GAME_DURATION_SECONDS;
  }

  // get time until next shift
  public double getTimeToNextShift () {
    double timeSinceTele = gameTimer.get() - Constants.FieldConstants.AUTO_LENGTH_IN_TIME;

    return Constants.FieldConstants.TIME_BETWEEN_SHIFTS - 
      ( timeSinceTele % Constants.FieldConstants.TIME_BETWEEN_SHIFTS);
  }

  /** Publishes shared data so subsystems (e.g. Dashboard) can read it. Call from robotPeriodic. */
  public void updateSharedData() {
    PowerRobotContainer.setData("timeToShift", getTimeToNextShift());
  }
}