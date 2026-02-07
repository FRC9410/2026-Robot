// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PositionSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VelocitySubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.MotorCancoderRequest;

public class RobotContainer {
  private Map<String, Object> subsystemData = new HashMap<>();
  public static final BiConsumer<String, Object> UPDATE_DATA = ((k, v) -> updateSubsystemData(k, v));

  private final LED led = new LED();
  private final Turret turret = new Turret();
  private final Shooter shooter = new Shooter();
  private final ShooterHood shooterHood = new ShooterHood();
  private final IntakeRoller intakeRoller = new IntakeRoller();
  private final IntakeWrist intakeWrist = new IntakeWrist();
  private final Spindexer spindexer = new Spindexer();
  private final VelocitySubsystem feeder = new VelocitySubsystem(Constants.FeederConstants.MOTOR_CONFIGS, UPDATE_DATA);
  private final Vision vision = new Vision();
  private final StateMachine stateMachine = new StateMachine();
  private final HashMap<String, Object> info = new HashMap<>();

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

  public IntakeRoller getIntakeRoller() {
    return intakeRoller;
  }

  public IntakeWrist getIntakeWrist() {
    return intakeWrist;
  }

  public Spindexer getSpindexer() {
    return spindexer;
  }

  public VelocitySubsystem getFeeder() {
    return feeder;
  }

  public StateMachine getStateMachine() {
    return stateMachine;
  }

  public void updateSubsystemData(String key, Object value) {
    subsystemData.put(key, value);
  }
  
  public static void updateData (String k, Object v) {
    subsystemData.put(k, v);
  }
}
