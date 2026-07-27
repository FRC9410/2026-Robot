// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/**
 * One camera's view of the world for one loop, decoded from the Limelight botpose array.
 *
 * <p>Deliberately free of NetworkTables and hardware types so the scoring and acceptance logic in
 * {@link VisionScoring} can be unit tested without a robot. Pose components are kept as raw doubles
 * rather than a {@code Pose2d} so a rejected sample never constructs one.
 *
 * @param connected false when the camera has never published or has gone silent
 * @param freshFrame true only on the loop a new frame actually arrived
 * @param maxAmbiguity worst per-tag ambiguity in the solution; 0 when unavailable
 * @param timestampSeconds latency-corrected FPGA timestamp, seconds
 * @param trust per-camera weight from {@link VisionConfig.CameraConfig}
 */
public record CameraSample(
    String name,
    boolean connected,
    boolean freshFrame,
    double x,
    double y,
    double yawDegrees,
    int tagCount,
    double tagSpan,
    double avgTagDist,
    double avgTagArea,
    double maxAmbiguity,
    double timestampSeconds,
    double trust) {

  /** A camera that is present but currently sees nothing usable. */
  public static CameraSample noTargets(String name, boolean connected, boolean freshFrame, double trust) {
    return new CameraSample(name, connected, freshFrame, 0, 0, 0, 0, 0, 0, 0, 0, 0, trust);
  }

  /** A camera that has never published, or has gone silent long enough to be considered gone. */
  public static CameraSample disconnected(String name, double trust) {
    return noTargets(name, false, false, trust);
  }

  public boolean hasTarget() {
    return connected && tagCount > 0;
  }

  /** True when two or more tags contributed, which is a far better conditioned solve. */
  public boolean isMultiTag() {
    return tagCount >= 2;
  }

  /** Same sample carried into a loop where no new frame arrived. */
  public CameraSample asStale(boolean stillConnected) {
    return new CameraSample(
        name, stillConnected, false, x, y, yawDegrees,
        tagCount, tagSpan, avgTagDist, avgTagArea, maxAmbiguity, timestampSeconds, trust);
  }
}
