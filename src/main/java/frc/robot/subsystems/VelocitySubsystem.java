// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VelocitySubsystem extends Subsystem9410 {

  /** Primary velocity-controlled motor; set in subclass after init. */
  protected TalonFX velocityMotor;

  public VelocitySubsystem() {
    super();
  }

  @Override
  public void periodic() {}

  /**
   * Initializes a TalonFX with PID and Motion Magic velocity config and registers it with the
   * parent. Uses this subsystem's CAN bus. Call from subclass constructor.
   *
   * @param request configuration (CAN id, PID gains, stop voltage, MM acceleration)
   * @return the configured TalonFX; use {@link #setVelocity(double)} or MotionMagicVelocityVoltage
   */
  protected TalonFX initAndRegisterVelocityMotor(VelocityConfigRequest request) {
    TalonFX motor = initFromRequest(request);
    registerMotor(request.canId(), motor);
    this.velocityMotor = motor;
    return motor;
  }

  /**
   * Initializes a TalonFX with PID and Motion Magic velocity config from a request (standalone;
   * does not register). Use when you need to build a motor without extending, or register via
   * {@link Subsystem9410#registerMotor(int, TalonFX)}.
   *
   * @param request configuration (CAN id, bus, PID gains, stop voltage, MM acceleration)
   * @return the configured TalonFX; use MotionMagicVelocityVoltage for velocity setpoints
   */
  public static TalonFX initFromRequest(VelocityConfigRequest request) {
    TalonFX motor = new TalonFX(request.canId(), request.canBus());

    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.setVoltage(request.stopVoltage());

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = request.kP();
    config.Slot0.kI = request.kI();
    config.Slot0.kD = request.kD();
    config.Slot0.kG = request.kG();

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.withMotionMagicAcceleration(request.mmAcceleration());

    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(motionMagicConfigs);
    motor.setNeutralMode(NeutralModeValue.Brake);

    return motor;
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
