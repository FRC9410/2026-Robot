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

  // --- Other ---
  private final StateMachine stateMachine = new StateMachine();

  /** Game timer: counts up from 0 to 2 minutes 40 seconds (160 s). Start via {@link #startGameTimer()}. */
  public static final double GAME_DURATION_SECONDS = 2 * 60 + 40; // 2:40
  private final Timer gameTimer = new Timer();

  // Controller
  private final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandXboxController TestController = new CommandXboxController(5);

  public RobotContainer() {
    configureBindings();
    configureTestBindings();

  }

  private void configureBindings() {
    

  }

  private void configureTestBindings() {
    TestController.a().toggleOnTrue(new InstantCommand(
      () -> stateMachine.spindexer.setVelocity(24)
    ));
     TestController.b().toggleOnTrue(new InstantCommand(
      () -> stateMachine.feeder.setVelocity(24)
    ));
     TestController.y().toggleOnTrue(new InstantCommand(
      () -> stateMachine.shooter.setVelocity(24)
    ));
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
}
