// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;

/**
 * Subsystem9410: extensible subsystem with helpers for controlling devices by CAN ID.
 * Use {@link #registerMotor(int)}, {@link #setOutput(int, double)}, and related helpers
 * to add and control TalonFX devices without duplicating setup code.
 */
public class Subsystem9410 extends SubsystemBase {

  private final CANBus bus;
  private final Map<Integer, TalonFX> motorsByCanId;

  /**
   * Constructor for subclasses that register their own motors (e.g. velocity/position configured).
   *
   */
  protected Subsystem9410() {
    this.bus = Constants.CanBusConstants.CANIVORE_BUS;
    this.motorsByCanId = new HashMap<>();
  }

  /** Returns the CAN bus used by this subsystem (for subclasses that build custom devices). */
  protected CANBus getBus() {
    return bus;
  }

  // ---------- CAN ID helpers (extensibility) ----------

  /**
   * Creates a TalonFX on the subsystem's CAN bus. Use this or {@link #createTalonFx(int,
   * NeutralModeValue)} when building devices for registration.
   */
  public TalonFX createTalonFx(int canId) {
    return new TalonFX(canId, bus);
  }

  /**
   * Creates a TalonFX on the subsystem's CAN bus with the given neutral mode.
   */
  public TalonFX createTalonFx(int canId, NeutralModeValue neutralMode) {
    TalonFX motor = createTalonFx(canId);
    motor.setNeutralMode(neutralMode);
    return motor;
  }

  /**
   * Registers a new TalonFX by CAN ID (creates it on the default bus with brake neutral mode).
   * Enables control via {@link #setOutput(int, double)}, {@link #stop(int)}, {@link #getMotor(int)}.
   */
  public TalonFX registerMotor(int canId) {
    return registerMotor(canId, NeutralModeValue.Brake);
  }

  /**
   * Registers a new TalonFX by CAN ID with the given neutral mode.
   */
  public TalonFX registerMotor(int canId, NeutralModeValue neutralMode) {
    TalonFX motor = createTalonFx(canId, neutralMode);
    motorsByCanId.put(canId, motor);
    return motor;
  }

  /**
   * Registers an existing TalonFX under a CAN ID so it can be controlled by {@link #setOutput(int,
   * double)} and other helpers.
   */
  public void registerMotor(int canId, TalonFX motor) {
    motorsByCanId.put(canId, motor);
  }

  /**
   * Configures a motor as a follower of another (by leader CAN ID). Leader must already be
   * registered or exist on the bus.
   */
  public void setFollower(int followerCanId, int leaderCanId) {
    TalonFX follower = getMotor(followerCanId);
    if (follower != null) {
      follower.setControl(
          new Follower(leaderCanId, MotorAlignmentValue.Aligned));
    }
  }

  /** Returns the TalonFX registered for the given CAN ID, or null if not registered. */
  public TalonFX getMotor(int canId) {
    return motorsByCanId.get(canId);
  }

  /** Sets percent output for the device at the given CAN ID (if registered). */
  public void setOutput(int canId, double output) {
    TalonFX motor = motorsByCanId.get(canId);
    if (motor != null) {
      motor.set(output);
    }
  }

  /** Stops the device at the given CAN ID (if registered). */
  public void stop(int canId) {
    setOutput(canId, 0);
  }

  /** Stops all registered motors. */
  public void stopAll() {
    for (TalonFX motor : motorsByCanId.values()) {
      motor.set(0);
    }
  }

  /** Sets neutral mode for a registered motor. */
  public void setNeutralMode(int canId, NeutralModeValue mode) {
    TalonFX motor = motorsByCanId.get(canId);
    if (motor != null) {
      motor.setNeutralMode(mode);
    }
  }

  /** Whether a motor is registered for the given CAN ID. */
  public boolean hasMotor(int canId) {
    return motorsByCanId.containsKey(canId);
  }

  @Override
  public void periodic() {}
}
