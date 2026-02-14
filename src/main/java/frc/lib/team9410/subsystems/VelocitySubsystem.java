// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team9410.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.team9410.PowerRobotContainer;
import frc.lib.team9410.configs.LeadMotorConfig;
import frc.lib.team9410.configs.MotionMagicConfig;
import frc.lib.team9410.configs.MotorConfig;


public class VelocitySubsystem extends PowerSubsystem {

  /** Primary velocity-controlled motor; set in subclass after init. */
  protected TalonFX velocityMotor;
  private String subsystemName;

  /**
   * Constructor that uses the leader motor already registered by the base from {@code config},
   * and configures it with the given lead and motion magic configs.
   *
   * @param config motor configs (first non-follower is the leader)
   * @param leadConfig PID gains for the leader (ratios unused for velocity)
   * @param motionMagicConfig motion magic acceleration (cruise velocity unused)
   */
  public VelocitySubsystem(
      List<MotorConfig> config,
      LeadMotorConfig leadConfig,
      MotionMagicConfig motionMagicConfig,
      String subsystemName) {
    super(config, subsystemName);
    TalonFX leader = getLeaderMotor();
    if (leader != null) {
      configureMotorForVelocity(leader, leadConfig, motionMagicConfig);
      this.velocityMotor = leader;
    }
    this.subsystemName = subsystemName;
  }

  @Override
  public void periodic() {
    SignalLogger.writeDouble(subsystemName + " Velocity", getLeaderMotor().getRotorVelocity().getValueAsDouble(), "rotations per second");
  }

  /**
   * Applies lead and motion magic config to an existing TalonFX for velocity control.
   */
  private static void configureMotorForVelocity(
      TalonFX motor,
      LeadMotorConfig leadConfig,
      MotionMagicConfig motionMagicConfig) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = leadConfig.kP();
    config.Slot0.kI = leadConfig.kI();
    config.Slot0.kD = leadConfig.kD();
    config.Slot0.kG = leadConfig.kG();

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.withMotionMagicAcceleration(motionMagicConfig.acceleration());

    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(motionMagicConfigs);
  }

  /**
   * Sets velocity setpoint (e.g. rotations per second) using Motion Magic. Override or use
   * directly after {@link #velocityMotor} is set.
   */
  public void setVelocity(double velocityRotationsPerSecond) {
    if (velocityMotor != null) {
      velocityMotor.setControl(
          new MotionMagicVelocityVoltage(0).withVelocity(velocityRotationsPerSecond).withSlot(0));
    }
  }

  /** Stops the velocity motor. */
  public void stopVelocity() {
    setVelocity(0);
  }

  /** Returns the primary velocity motor (if initialized). */
  public TalonFX getVelocityMotor() {
    return velocityMotor;
  }
}