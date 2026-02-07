package frc.robot.util;

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
import static edu.wpi.first.units.Units.Rotations;

import frc.robot.Constants;

public class EncoderHelpers {

    /**
     * Initializes a TalonFX + CANcoder pair with PID and Motion Magic from a request.
     *
     * @param request configuration (motor/encoder IDs, PID gains, ratios, motion magic, encoder config)
     * @return the configured TalonFX (with fused CANcoder); callers use Motion Magic for position control
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
}
