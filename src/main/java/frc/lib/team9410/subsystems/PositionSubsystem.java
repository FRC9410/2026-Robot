// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team9410.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;
import frc.lib.team9410.configs.CancoderConfig;
import frc.lib.team9410.configs.LeadMotorConfig;
import frc.lib.team9410.configs.MotionMagicConfig;
import frc.lib.team9410.configs.MotorConfig;

public class PositionSubsystem extends PowerSubsystem {

  /** Primary position-controlled motor (with fused CANcoder from config constructor). */
  protected TalonFX positionMotor;
  private String subsystemName;
  private String units;

  /**
   * Constructor that uses the leader motor already registered by the base from {@code config},
   * and configures it with the given lead, CANcoder, and motion magic configs.
   *
   * @param config motor configs (first non-follower is the leader)
   * @param leadConfig PID and feedback ratios for the leader
   * @param cancoderConfig CANcoder ID and magnet settings
   * @param motionMagicConfig cruise velocity and acceleration
   */
  public PositionSubsystem(
      List<MotorConfig> config,
      LeadMotorConfig leadConfig,
      CancoderConfig cancoderConfig,
      MotionMagicConfig motionMagicConfig,
      String subsystemName,
      String units) {
    super(config, subsystemName);
    TalonFX leader = getLeaderMotor();
    if (leader != null) {
      configureMotorWithCancoder(leader, leadConfig, cancoderConfig, motionMagicConfig);
      this.positionMotor = leader;
    }
    this.subsystemName = subsystemName;
    this.units = units;
  }

  @Override
  public void periodic() {
    SignalLogger.writeDouble(subsystemName + " Position", getLeaderMotor().getPosition().getValueAsDouble(), units);
  }

  /**
   * Configures an existing TalonFX with a CANcoder and the given lead, CANcoder, and motion magic
   * configs. Used by the config-list constructor.
   */
  private static void configureMotorWithCancoder(
      TalonFX motor,
      LeadMotorConfig leadConfig,
      CancoderConfig cancoderConfig,
      MotionMagicConfig motionMagicConfig) {
    @SuppressWarnings("resource") // CANcoder is fused to motor, lifecycle tied to subsystem
    CANcoder cancoder =
        new CANcoder(cancoderConfig.encoderId(), Constants.CanBusConstants.CANIVORE_BUS);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(
        Rotations.of(cancoderConfig.discontinuityPointRotations()));
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(cancoderConfig.magnetOffsetRotations()));
    cancoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.Slot0.kP = leadConfig.kP();
    talonConfig.Slot0.kI = leadConfig.kI();
    talonConfig.Slot0.kD = leadConfig.kD();
    talonConfig.Slot0.kG = leadConfig.kG();
    talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonConfig.Feedback.SensorToMechanismRatio = leadConfig.sensorToMechanismRatio();
    talonConfig.Feedback.RotorToSensorRatio = leadConfig.rotorToSensorRatio();

    motor.getConfigurator().apply(talonConfig);

    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs
        .withMotionMagicCruiseVelocity(motionMagicConfig.cruiseVelocity())
        .withMotionMagicAcceleration(motionMagicConfig.acceleration());
    motor.getConfigurator().apply(mmConfigs);

    BaseStatusSignal.setUpdateFrequencyForAll(100, cancoder.getPosition(), cancoder.getVelocity());
    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.setControl(new MotionMagicVoltage(0).withPosition(0.07).withSlot(0));
  }

  /**
   * Sets position setpoint in rotations using Motion Magic. Override or use directly after {@link
   * #positionMotor} is set.
   */
  public void setPositionRotations(double rotations) {
    if (positionMotor != null) {
      positionMotor.setControl(new MotionMagicVoltage(0).withPosition(rotations).withSlot(0));
    }
  }

  /**
   * Sets position setpoint in degrees (converted to rotations). Override or use directly after
   * {@link #positionMotor} is set.
   */
  public void setPositionDegrees(double degrees) {
    setPositionRotations(degrees / 360.0);
  }

  /** Stops the position motor (zero demand). */
  public void stopPosition() {
    if (positionMotor != null) {
      positionMotor.set(0);
    }
  }

  /** Returns position in rotations from the primary position motor (if initialized). */
  public double getPositionRotations() {
    return positionMotor != null ? positionMotor.getPosition().getValueAsDouble() : 0;
  }

  /** Returns position in degrees from the primary position motor (if initialized). */
  public double getPositionDegrees() {
    return getPositionRotations() * 360.0;
  }

  /** Returns the primary position motor (if initialized). */
  public TalonFX getPositionMotor() {
    return positionMotor;
  }
}
