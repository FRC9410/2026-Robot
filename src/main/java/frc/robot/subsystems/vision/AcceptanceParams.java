// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/**
 * Thresholds deciding whether a {@link CameraSample} is trustworthy enough to hand to the pose
 * estimator. Single-tag solutions get much tighter gates than multi-tag ones: a single tag has no
 * baseline, so its solve is poorly conditioned and its pose can flip.
 *
 * @param maxSingleTagDist max average tag distance, meters, when only one tag is visible
 * @param maxMultiTagDist max average tag distance, meters, with two or more tags
 * @param maxSingleTagAmbiguity max per-tag ambiguity tolerated for a single-tag solution
 * @param minSingleTagArea min average tag area (fraction of image) for a single-tag solution
 * @param maxJumpSingleTag max distance, meters, a single-tag pose may sit from the current estimate
 * @param maxJumpMultiTag max distance, meters, a multi-tag pose may sit from the current estimate
 */
public record AcceptanceParams(
    double maxSingleTagDist,
    double maxMultiTagDist,
    double maxSingleTagAmbiguity,
    double minSingleTagArea,
    double maxJumpSingleTag,
    double maxJumpMultiTag) {

  public static AcceptanceParams defaults() {
    return new AcceptanceParams(4.0, 6.5, 0.3, 0.08, 1.0, 2.5);
  }
}
