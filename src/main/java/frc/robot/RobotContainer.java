// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  private final LED led = new LED();
  private final Turret turret = new Turret();
  private final Shooter shooter = new Shooter();
  private final ShooterHood shooterHood = new ShooterHood();
  private final Vision vision = new Vision();
  private final StateMachine stateMachine = new StateMachine();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public LED getLED() {
    return led;
  }

  public Turret getTurret() {
    return turret;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public ShooterHood getShooterHood() {
    return shooterHood;
  }

  public Vision getVision() {
    return vision;
  }

  public StateMachine getStateMachine() {
    return stateMachine;
  }
}
