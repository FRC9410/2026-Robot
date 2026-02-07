// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that uses Motion Magic velocity control. Create a motor with {@link
 * #initFromRequest(VelocityConfigRequest)} in your subclass constructor.
 */
public class VelocitySubsystem extends SubsystemBase {

  public VelocitySubsystem() {}

  @Override
  public void periodic() {}

  /**
   * Initializes a TalonFX with PID and Motion Magic velocity config from a request.
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
}
