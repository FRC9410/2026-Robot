// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

/** No HAL, no NetworkTables -- everything under test is a pure function. */
class VisionScoringTest {

  private static final AcceptanceParams PARAMS = AcceptanceParams.defaults();
  private static final StdDevParams STD_DEVS = StdDevParams.defaults();
  private static final FieldBounds BOUNDS = new FieldBounds(0.0, 0.0, 17.5, 8.0, 0.4);

  /** A fresh, connected sample in the middle of the field. */
  private static CameraSample sample(
      String name, int tagCount, double area, double dist, double ambiguity) {
    return sample(name, tagCount, area, dist, ambiguity, 8.0, 4.0, 1.0);
  }

  private static CameraSample sample(
      String name,
      int tagCount,
      double area,
      double dist,
      double ambiguity,
      double x,
      double y,
      double trust) {
    return new CameraSample(
        name, true, true, x, y, 0.0, tagCount, 1.0, dist, area, ambiguity, 100.0, trust);
  }

  // --- score ---------------------------------------------------------------

  @Test
  @DisplayName("two distant tags beat one huge close tag -- the bug the old area-only code had")
  void multiTagBeatsSingleTagRegardlessOfArea() {
    CameraSample oneBigClose = sample("turret", 1, 0.90, 1.0, 0.05);
    CameraSample twoSmallFar = sample("left", 2, 0.05, 4.0, 0.05);

    assertTrue(VisionScoring.score(twoSmallFar) > VisionScoring.score(oneBigClose));
  }

  @Test
  void samplesWithoutTargetsAreUnusableAndNeverWin() {
    CameraSample blind = CameraSample.noTargets("left", true, true, 1.0);
    CameraSample offline = CameraSample.disconnected("right", 1.0);
    CameraSample worst = sample("turret", 1, 0.01, 6.0, 0.9);

    assertEquals(Double.NEGATIVE_INFINITY, VisionScoring.score(blind));
    assertEquals(Double.NEGATIVE_INFINITY, VisionScoring.score(offline));
    assertTrue(VisionScoring.score(worst) > VisionScoring.score(blind));
  }

  @Test
  void ambiguityDragsScoreDown() {
    CameraSample clean = sample("left", 1, 0.2, 2.0, 0.0);
    CameraSample ambiguous = sample("right", 1, 0.2, 2.0, 0.5);

    assertTrue(VisionScoring.score(clean) > VisionScoring.score(ambiguous));
  }

  @Test
  void closerIsBetterAllElseEqual() {
    CameraSample near = sample("left", 2, 0.2, 2.0, 0.0);
    CameraSample far = sample("right", 2, 0.2, 5.0, 0.0);

    assertTrue(VisionScoring.score(near) > VisionScoring.score(far));
  }

  @Test
  void identicalSamplesTieBreakByTrust() {
    CameraSample trusted = sample("left", 2, 0.2, 2.0, 0.0, 8.0, 4.0, 1.0);
    CameraSample doubted = sample("turret", 2, 0.2, 2.0, 0.0, 8.0, 4.0, 0.8);

    assertTrue(VisionScoring.score(trusted) > VisionScoring.score(doubted));
  }

  // --- isAcceptable --------------------------------------------------------

  @Test
  void staleFrameIsRejectedEvenWithGoodData() {
    CameraSample good = sample("left", 2, 0.3, 2.0, 0.0);

    assertTrue(VisionScoring.isAcceptable(good, PARAMS, BOUNDS, null, false));
    assertFalse(VisionScoring.isAcceptable(good.asStale(true), PARAMS, BOUNDS, null, false));
  }

  @Test
  void disconnectedCameraIsRejected() {
    assertFalse(
        VisionScoring.isAcceptable(
            CameraSample.disconnected("left", 1.0), PARAMS, BOUNDS, null, false));
  }

  @Test
  @DisplayName("the (0,0) default pose from a cold Limelight is rejected despite being in bounds")
  void originPoseIsRejected() {
    CameraSample atOrigin = sample("left", 2, 0.3, 2.0, 0.0, 0.0, 0.0, 1.0);

    assertTrue(BOUNDS.contains(0.0, 0.0));
    assertFalse(VisionScoring.isAcceptable(atOrigin, PARAMS, BOUNDS, null, false));
  }

  @Test
  void slightlyOutsideFieldIsAcceptedButFarOutsideIsNot() {
    CameraSample justOutside = sample("left", 2, 0.3, 2.0, 0.0, -0.3, 4.0, 1.0);
    CameraSample wayOutside = sample("left", 2, 0.3, 2.0, 0.0, -1.0, 4.0, 1.0);

    assertTrue(VisionScoring.isAcceptable(justOutside, PARAMS, BOUNDS, null, false));
    assertFalse(VisionScoring.isAcceptable(wayOutside, PARAMS, BOUNDS, null, false));
  }

  @Test
  void singleTagGatesAreTighterThanMultiTagGates() {
    // 5 m is past maxSingleTagDist (4.0) but inside maxMultiTagDist (6.5).
    CameraSample oneFar = sample("left", 1, 0.3, 5.0, 0.0);
    CameraSample twoFar = sample("left", 2, 0.3, 5.0, 0.0);

    assertFalse(VisionScoring.isAcceptable(oneFar, PARAMS, BOUNDS, null, false));
    assertTrue(VisionScoring.isAcceptable(twoFar, PARAMS, BOUNDS, null, false));
  }

  @Test
  void ambiguousSingleTagIsRejectedButTheSameTagCleanIsAccepted() {
    CameraSample ambiguous = sample("left", 1, 0.3, 2.0, 0.5);
    CameraSample clean = sample("left", 1, 0.3, 2.0, 0.1);

    assertFalse(VisionScoring.isAcceptable(ambiguous, PARAMS, BOUNDS, null, false));
    assertTrue(VisionScoring.isAcceptable(clean, PARAMS, BOUNDS, null, false));
  }

  @Test
  void tinySingleTagIsRejectedOnArea() {
    CameraSample tiny = sample("left", 1, 0.01, 2.0, 0.0);

    assertFalse(VisionScoring.isAcceptable(tiny, PARAMS, BOUNDS, null, false));
  }

  @Test
  void teleportGateRejectsBigJumpsForSingleTagButAllowsModerateOnesForMultiTag() {
    Pose2d current = new Pose2d(8.0, 4.0, new edu.wpi.first.math.geometry.Rotation2d());
    CameraSample singleFarAway = sample("left", 1, 0.3, 2.0, 0.0, 13.0, 4.0, 1.0); // 5 m jump
    CameraSample multiNearby = sample("left", 2, 0.3, 2.0, 0.0, 10.4, 4.0, 1.0); // 2.4 m jump

    assertFalse(VisionScoring.isAcceptable(singleFarAway, PARAMS, BOUNDS, current, true));
    assertTrue(VisionScoring.isAcceptable(multiNearby, PARAMS, BOUNDS, current, true));
  }

  @Test
  @DisplayName("teleport gate is disabled before the estimate is seeded")
  void teleportGateIgnoredWhenPoseNotYetSeeded() {
    Pose2d current = new Pose2d(8.0, 4.0, new edu.wpi.first.math.geometry.Rotation2d());
    CameraSample farAway = sample("left", 1, 0.3, 2.0, 0.0, 13.0, 4.0, 1.0);

    assertFalse(VisionScoring.isAcceptable(farAway, PARAMS, BOUNDS, current, true));
    assertTrue(VisionScoring.isAcceptable(farAway, PARAMS, BOUNDS, current, false));
  }

  // --- xyStdDev ------------------------------------------------------------

  @Test
  void stdDevGrowsWithDistanceAndShrinksWithTagCount() {
    double oneNear = VisionScoring.xyStdDev(sample("a", 1, 0.3, 2.0, 0.0), STD_DEVS);
    double oneFar = VisionScoring.xyStdDev(sample("a", 1, 0.3, 4.0, 0.0), STD_DEVS);
    double twoNear = VisionScoring.xyStdDev(sample("a", 2, 0.3, 2.0, 0.0), STD_DEVS);

    assertTrue(oneFar > oneNear, "farther must be trusted less");
    assertTrue(twoNear < oneNear, "more tags must be trusted more");
  }

  @Test
  void stdDevIsClampedAtBothEnds() {
    double veryClose = VisionScoring.xyStdDev(sample("a", 4, 0.9, 0.1, 0.0), STD_DEVS);
    double veryFar = VisionScoring.xyStdDev(sample("a", 1, 0.01, 40.0, 0.0), STD_DEVS);

    assertEquals(STD_DEVS.minXy(), veryClose);
    assertEquals(STD_DEVS.maxXy(), veryFar);
  }

  @Test
  void lowerTrustProducesLargerStdDev() {
    double trusted = VisionScoring.xyStdDev(sample("a", 2, 0.3, 3.0, 0.0, 8, 4, 1.0), STD_DEVS);
    double doubted = VisionScoring.xyStdDev(sample("a", 2, 0.3, 3.0, 0.0, 8, 4, 0.5), STD_DEVS);

    assertTrue(doubted > trusted);
  }

  @Test
  @DisplayName("theta std dev is finite -- infinity risks NaN in the estimator's Kalman gain")
  void thetaStdDevIsFinite() {
    assertTrue(Double.isFinite(STD_DEVS.theta()));
    assertTrue(STD_DEVS.theta() > 1e6);
  }
}
