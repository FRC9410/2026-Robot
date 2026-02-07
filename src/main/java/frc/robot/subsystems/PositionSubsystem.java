// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.List;
import java.util.function.BiConsumer;

import com.ctre.phoenix6.BaseStatusSignal;
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
import frc.robot.RobotContainer9410;
import frc.robot.util.MotorCancoderRequest;
import frc.robot.io.MotorConfig;

public class PositionSubsystem extends Subsystem9410 {

  /** Primary position-controlled motor (with fused CANcoder when using initAndRegisterPositionMotor). */
  protected TalonFX positionMotor;
  

  public PositionSubsystem(List<MotorConfig> config) {
    super(config);
  }

  @Override
  public void periodic() {}

  /**
   * Initializes a TalonFX + CANcoder pair with PID and Motion Magic from a request.
   * Use from any subsystem that needs position control with an external CANcoder.
   *
   * @param request configuration (motor/encoder IDs, PID gains, ratios, motion magic, encoder config)
   * @return the configured TalonFX (with fused CANcoder); use MotionMagicVoltage for position setpoints
   */
  public static TalonFX initMotorCancoderPair(MotorCancoderRequest request) {
    TalonFX motor =
        new TalonFX(request.motorId(), Constants.CanBusConstants.CANIVORE_BUS);

    @SuppressWarnings("resource") // CANcoder is fused to motor, lifecycle tied to subsystem
    CANcoder cancoder =
        new CANcoder(request.encoderId(), Constants.CanBusConstants.CANIVORE_BUS);

    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(
        Rotations.of(request.discontinuityPointRotations()));
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(request.magnetOffsetRotations()));

    talonConfig.Slot0.kP = request.kP();
    talonConfig.Slot0.kI = request.kI();
    talonConfig.Slot0.kD = request.kD();
    talonConfig.Slot0.kG = request.kG();
    talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonConfig.Feedback.SensorToMechanismRatio = request.sensorToMechanismRatio();
    talonConfig.Feedback.RotorToSensorRatio = request.rotorToSensorRatio();

    motor.getConfigurator().apply(talonConfig);

    mmConfigs
        .withMotionMagicCruiseVelocity(request.mmCruiseVelocity())
        .withMotionMagicAcceleration(request.mmAcceleration());

    motor.getConfigurator().apply(mmConfigs);

    BaseStatusSignal.setUpdateFrequencyForAll(100, cancoder.getPosition(), cancoder.getVelocity());
    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.setControl(new MotionMagicVoltage(0).withPosition(0.07).withSlot(0));

    return motor;
  }

  /**
   * Initializes a TalonFX + CANcoder pair with PID and Motion Magic from a request, and registers
   * the motor with the parent. Call from subclass constructor.
   *
   * @param request configuration (motor/encoder IDs, PID, ratios, motion magic, encoder config)
   * @return the configured TalonFX; use {@link #setPositionRotations(double)} for setpoints
   */
  protected TalonFX initAndRegisterPositionMotor(MotorCancoderRequest request) {
    TalonFX motor = initMotorCancoderPair(request);
    registerMotor(request.motorId(), motor);
    this.positionMotor = motor;
    return motor;
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
