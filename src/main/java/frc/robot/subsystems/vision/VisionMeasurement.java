// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * An accepted vision pose, ready to hand to a pose estimator. Standard deviations travel with the
 * measurement so several cameras with different trust can be applied in the same loop.
 *
 * @param timestampSeconds raw FPGA seconds. Callers must NOT convert this -- {@code Swerve}
 *     already applies {@code Utils.fpgaToCurrentTime} inside its {@code addVisionMeasurement}
 *     override, and converting here too would double-apply the offset.
 * @param thetaStdDev radians; pinned huge for MegaTag2, whose heading is the one we supplied
 */
public record VisionMeasurement(
    String cameraName,
    Pose2d pose,
    double timestampSeconds,
    int tagCount,
    double avgTagDist,
    double xyStdDev,
    double thetaStdDev) {}
