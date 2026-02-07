package frc.robot.util;

/**
 * Request object for initializing a TalonFX + CANcoder pair with PID and Motion Magic.
 * Use with {@link EncoderHelpers#initMotorCancoderPair(MotorCancoderRequest)}.
 */
public record MotorCancoderRequest(
    int motorId,
    int encoderId,
    int kP,
    int kI,
    int kD,
    int kG,
    int sensorToMechanismRatio,
    int rotorToSensorRatio,
    int mmCruiseVelocity,
    int mmAcceleration,
    double magnetOffsetRotations,
    double discontinuityPointRotations
) {
  /** Default magnet offset and discontinuity if not specified. */
  public static final double DEFAULT_MAGNET_OFFSET_ROTATIONS = -0.1;
  public static final double DEFAULT_DISCONTINUITY_POINT_ROTATIONS = 1.0;

  /**
   * Creates a request with default encoder config (magnet offset and discontinuity point).
   */
  public static MotorCancoderRequest withDefaults(
      int motorId,
      int encoderId,
      int kP,
      int kI,
      int kD,
      int kG,
      int sensorToMechanismRatio,
      int rotorToSensorRatio,
      int mmCruiseVelocity,
      int mmAcceleration) {
    return new MotorCancoderRequest(
        motorId,
        encoderId,
        kP,
        kI,
        kD,
        kG,
        sensorToMechanismRatio,
        rotorToSensorRatio,
        mmCruiseVelocity,
        mmAcceleration,
        DEFAULT_MAGNET_OFFSET_ROTATIONS,
        DEFAULT_DISCONTINUITY_POINT_ROTATIONS);
  }
}
