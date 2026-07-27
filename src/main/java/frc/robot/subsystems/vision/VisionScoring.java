// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Pure decision logic for vision measurements: is this sample trustworthy, how good is it relative
 * to the others, and how much should the pose estimator believe it.
 *
 * <p>No NetworkTables, no HAL, no state -- every method is a function of its arguments, which is
 * what makes this the part worth unit testing and the part to tune when vision misbehaves.
 */
public final class VisionScoring {

  private VisionScoring() {}

  /** Below this, distance terms stop being meaningful and start blowing up the trust curve. */
  private static final double MIN_MEANINGFUL_DIST_METERS = 0.5;

  /**
   * Relative quality of a sample. Higher is better; {@link Double#NEGATIVE_INFINITY} means unusable
   * so an unusable camera can never win a comparison.
   *
   * <p>The multi-tag bonus dominates every other term on purpose. Two distant tags give a far
   * better conditioned solve than one enormous close one, which is exactly the case the old
   * area-only comparison got backwards.
   */
  public static double score(CameraSample s) {
    if (!s.hasTarget()) {
      return Double.NEGATIVE_INFINITY;
    }
    double dist = Math.max(s.avgTagDist(), MIN_MEANINGFUL_DIST_METERS);
    double raw =
        (s.isMultiTag() ? 100.0 : 0.0)
            + 15.0 * Math.min(s.tagCount(), 4)
            + 8.0 * s.tagSpan()
            + 20.0 * Math.sqrt(Math.max(s.avgTagArea(), 0.0))
            - 6.0 * dist
            - 30.0 * s.maxAmbiguity();
    return raw * s.trust();
  }

  /**
   * Whether a sample may be handed to the pose estimator.
   *
   * @param current current pose estimate, for the teleport gate
   * @param havePose false before the estimate has been seeded, which disables the teleport gate
   */
  public static boolean isAcceptable(
      CameraSample s,
      AcceptanceParams params,
      FieldBounds bounds,
      Pose2d current,
      boolean havePose) {
    if (!s.hasTarget() || !s.freshFrame()) {
      return false;
    }
    // A cold or absent Limelight reports exactly (0, 0), which is inside the field and would
    // otherwise sail through the bounds check.
    if (s.x() == 0.0 && s.y() == 0.0) {
      return false;
    }
    if (!bounds.contains(s.x(), s.y())) {
      return false;
    }

    boolean multi = s.isMultiTag();
    double maxDist = multi ? params.maxMultiTagDist() : params.maxSingleTagDist();
    if (s.avgTagDist() > maxDist) {
      return false;
    }
    if (!multi
        && (s.maxAmbiguity() > params.maxSingleTagAmbiguity()
            || s.avgTagArea() < params.minSingleTagArea())) {
      return false;
    }

    if (havePose && current != null) {
      double jump = Math.hypot(s.x() - current.getX(), s.y() - current.getY());
      double maxJump = multi ? params.maxJumpMultiTag() : params.maxJumpSingleTag();
      if (jump > maxJump) {
        return false;
      }
    }
    return true;
  }

  /**
   * Translational standard deviation for the pose estimator: trust falls off with the square of
   * distance and improves with tag count, which is the standard formulation. A camera configured
   * with lower trust gets a proportionally larger deviation.
   */
  public static double xyStdDev(CameraSample s, StdDevParams params) {
    double dist = Math.max(s.avgTagDist(), MIN_MEANINGFUL_DIST_METERS);
    double raw =
        params.baseXy()
            * dist
            * dist
            / Math.max(s.tagCount(), 1)
            / Math.max(s.trust(), 1e-3);
    return MathUtil.clamp(raw, params.minXy(), params.maxXy());
  }
}
