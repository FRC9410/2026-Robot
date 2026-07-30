// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.subsystems.Vision.AcceptanceParams;
import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.subsystems.Vision.CameraSample;
import frc.robot.subsystems.Vision.FieldBounds;
import frc.robot.subsystems.Vision.StdDevParams;
import frc.robot.subsystems.Vision.Verdict;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

// No HAL, no NetworkTables -- everything under test is a pure function.
class VisionTest {

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

  private static Verdict verdictOf(CameraSample s) {
    return Vision.verdict(s, PARAMS, BOUNDS, null, false);
  }

  // --- score ---------------------------------------------------------------

  @Test
  @DisplayName("two distant tags beat one huge close tag -- the bug the old area-only code had")
  void multiTagBeatsSingleTagRegardlessOfArea() {
    CameraSample oneBigClose = sample("turret", 1, 0.90, 1.0, 0.05);
    CameraSample twoSmallFar = sample("left", 2, 0.05, 4.0, 0.05);

    assertTrue(Vision.score(twoSmallFar) > Vision.score(oneBigClose));
  }

  @Test
  void samplesWithoutTargetsAreUnusableAndNeverWin() {
    CameraSample blind = CameraSample.noTargets("left", true, true, 1.0);
    CameraSample offline = CameraSample.disconnected("right", 1.0);
    CameraSample worst = sample("turret", 1, 0.01, 6.0, 0.9);

    assertEquals(Double.NEGATIVE_INFINITY, Vision.score(blind));
    assertEquals(Double.NEGATIVE_INFINITY, Vision.score(offline));
    assertTrue(Vision.score(worst) > Vision.score(blind));
  }

  @Test
  void ambiguityDragsScoreDown() {
    assertTrue(Vision.score(sample("l", 1, 0.2, 2.0, 0.0)) > Vision.score(sample("r", 1, 0.2, 2.0, 0.5)));
  }

  @Test
  void closerIsBetterAllElseEqual() {
    assertTrue(Vision.score(sample("l", 2, 0.2, 2.0, 0.0)) > Vision.score(sample("r", 2, 0.2, 5.0, 0.0)));
  }

  @Test
  void identicalSamplesTieBreakByTrust() {
    CameraSample trusted = sample("left", 2, 0.2, 2.0, 0.0, 8.0, 4.0, 1.0);
    CameraSample doubted = sample("turret", 2, 0.2, 2.0, 0.0, 8.0, 4.0, 0.8);

    assertTrue(Vision.score(trusted) > Vision.score(doubted));
  }

  // --- verdict -------------------------------------------------------------

  @Test
  void staleFrameIsRejectedEvenWithGoodData() {
    CameraSample good = sample("left", 2, 0.3, 2.0, 0.0);

    assertEquals(Verdict.ACCEPT, verdictOf(good));
    assertEquals(Verdict.STALE, verdictOf(good.asStale(true)));
  }

  @Test
  void disconnectedCameraIsRejected() {
    assertEquals(Verdict.NO_TARGET, verdictOf(CameraSample.disconnected("left", 1.0)));
  }

  @Test
  @DisplayName("the (0,0) default pose from a cold Limelight is rejected despite being in bounds")
  void originPoseIsRejected() {
    assertTrue(BOUNDS.contains(0.0, 0.0));
    assertEquals(Verdict.ORIGIN, verdictOf(sample("left", 2, 0.3, 2.0, 0.0, 0.0, 0.0, 1.0)));
  }

  @Test
  void slightlyOutsideFieldIsAcceptedButFarOutsideIsNot() {
    assertEquals(Verdict.ACCEPT, verdictOf(sample("l", 2, 0.3, 2.0, 0.0, -0.3, 4.0, 1.0)));
    assertEquals(Verdict.OUT_OF_BOUNDS, verdictOf(sample("l", 2, 0.3, 2.0, 0.0, -1.0, 4.0, 1.0)));
  }

  @Test
  void singleTagGatesAreTighterThanMultiTagGates() {
    // 5 m is past maxSingleTagDist (4.0) but inside maxMultiTagDist (6.5).
    assertEquals(Verdict.TOO_FAR, verdictOf(sample("l", 1, 0.3, 5.0, 0.0)));
    assertEquals(Verdict.ACCEPT, verdictOf(sample("l", 2, 0.3, 5.0, 0.0)));
  }

  @Test
  void ambiguousSingleTagIsRejectedButTheSameTagCleanIsAccepted() {
    assertEquals(Verdict.AMBIGUOUS, verdictOf(sample("l", 1, 0.3, 2.0, 0.5)));
    assertEquals(Verdict.ACCEPT, verdictOf(sample("l", 1, 0.3, 2.0, 0.1)));
  }

  @Test
  void tinySingleTagIsRejectedOnArea() {
    assertEquals(Verdict.TOO_SMALL, verdictOf(sample("l", 1, 0.01, 2.0, 0.0)));
  }

  // --- the jump gate and its escape hatch ----------------------------------

  @Test
  void jumpGateRejectsImplausibleSingleTagPoses() {
    Pose2d current = new Pose2d(8.0, 4.0, Rotation2d.kZero);
    CameraSample farAway = sample("left", 1, 0.3, 2.0, 0.0, 13.0, 4.0, 1.0); // 5 m jump

    assertEquals(Verdict.JUMPED, Vision.verdict(farAway, PARAMS, BOUNDS, current, true));
  }

  @Test
  @DisplayName("multi-tag is exempt from the jump gate, so drifted odometry can still recover")
  void multiTagIsExemptFromTheJumpGate() {
    Pose2d current = new Pose2d(8.0, 4.0, Rotation2d.kZero);
    CameraSample farAway = sample("left", 2, 0.3, 2.0, 0.0, 13.0, 4.0, 1.0); // 5 m jump

    assertEquals(Verdict.ACCEPT, Vision.verdict(farAway, PARAMS, BOUNDS, current, true));
  }

  @Test
  @DisplayName("jump gate is disabled before the estimate is seeded")
  void jumpGateIgnoredWhenPoseNotYetSeeded() {
    Pose2d current = new Pose2d(8.0, 4.0, Rotation2d.kZero);
    CameraSample farAway = sample("left", 1, 0.3, 2.0, 0.0, 13.0, 4.0, 1.0);

    assertEquals(Verdict.JUMPED, Vision.verdict(farAway, PARAMS, BOUNDS, current, true));
    assertEquals(Verdict.ACCEPT, Vision.verdict(farAway, PARAMS, BOUNDS, current, false));
  }

  // --- xyStdDev ------------------------------------------------------------

  @Test
  void stdDevGrowsWithDistanceAndShrinksWithTagCount() {
    double oneNear = Vision.xyStdDev(sample("a", 1, 0.3, 2.0, 0.0), STD_DEVS);
    double oneFar = Vision.xyStdDev(sample("a", 1, 0.3, 4.0, 0.0), STD_DEVS);
    double twoNear = Vision.xyStdDev(sample("a", 2, 0.3, 2.0, 0.0), STD_DEVS);

    assertTrue(oneFar > oneNear, "farther must be trusted less");
    assertTrue(twoNear < oneNear, "more tags must be trusted more");
  }

  @Test
  void stdDevIsClampedAtBothEnds() {
    assertEquals(STD_DEVS.minXy(), Vision.xyStdDev(sample("a", 4, 0.9, 0.1, 0.0), STD_DEVS));
    assertEquals(STD_DEVS.maxXy(), Vision.xyStdDev(sample("a", 1, 0.01, 40.0, 0.0), STD_DEVS));
  }

  @Test
  void lowerTrustProducesLargerStdDev() {
    double trusted = Vision.xyStdDev(sample("a", 2, 0.3, 3.0, 0.0, 8, 4, 1.0), STD_DEVS);
    double doubted = Vision.xyStdDev(sample("a", 2, 0.3, 3.0, 0.0, 8, 4, 0.5), STD_DEVS);

    assertTrue(doubted > trusted);
  }

  @Test
  @DisplayName("theta std dev is finite -- infinity risks NaN in the estimator's Kalman gain")
  void thetaStdDevIsFinite() {
    assertTrue(Double.isFinite(STD_DEVS.theta()));
    assertTrue(STD_DEVS.theta() > 1e6);
  }

  // --- decode --------------------------------------------------------------
  // An index error here produces poses that look entirely plausible and are wrong, so the layout
  // is pinned against LimelightHelpers v1.14 getBotPoseEstimate.

  private static final CameraConfig CAM = new CameraConfig("limelight-left", 0.8);

  // Builds a botpose array: 11 summary values then 7 per fiducial.
  private static TimestampedDoubleArray botpose(
      long ntTimestampMicros,
      double x,
      double y,
      double yawDeg,
      double latencyMs,
      int tagCount,
      double tagSpan,
      double avgDist,
      double avgArea,
      double... ambiguities) {
    double[] v = new double[11 + 7 * ambiguities.length];
    v[0] = x;
    v[1] = y;
    v[2] = 0.3; // z
    v[3] = 1.0; // roll
    v[4] = 2.0; // pitch
    v[5] = yawDeg;
    v[6] = latencyMs;
    v[7] = tagCount;
    v[8] = tagSpan;
    v[9] = avgDist;
    v[10] = avgArea;
    for (int i = 0; i < ambiguities.length; i++) {
      int base = 11 + i * 7;
      v[base] = 17 + i; // id
      v[base + 1] = 0.1; // txnc
      v[base + 2] = 0.2; // tync
      v[base + 3] = 0.3; // ta
      v[base + 4] = 2.0; // distToCamera
      v[base + 5] = 2.1; // distToRobot
      v[base + 6] = ambiguities[i];
    }
    return new TimestampedDoubleArray(ntTimestampMicros, ntTimestampMicros, v);
  }

  @Test
  void decodeReadsEveryFieldFromTheRightIndex() {
    CameraSample s =
        Vision.decode(botpose(5_000_000L, 3.25, 6.5, 42.0, 20.0, 2, 1.75, 2.5, 0.42, 0.1, 0.05), CAM);

    assertEquals("limelight-left", s.name());
    assertTrue(s.connected());
    assertTrue(s.freshFrame());
    assertEquals(3.25, s.x());
    assertEquals(6.5, s.y());
    assertEquals(42.0, s.yawDegrees());
    assertEquals(2, s.tagCount());
    assertEquals(1.75, s.tagSpan());
    assertEquals(2.5, s.avgTagDist());
    assertEquals(0.42, s.avgTagArea());
    assertEquals(0.8, s.trust());
  }

  @Test
  @DisplayName("timestamp is NT micros converted to seconds, minus the reported latency")
  void decodeSubtractsLatencyFromTheTimestamp() {
    CameraSample s = Vision.decode(botpose(5_000_000L, 3.0, 4.0, 0, 20.0, 1, 0, 2.0, 0.3, 0.0), CAM);

    assertEquals(5.0 - 0.020, s.timestampSeconds(), 1e-9);
  }

  @Test
  void decodeTakesTheWorstAmbiguityAcrossTags() {
    CameraSample s =
        Vision.decode(botpose(1_000_000L, 3.0, 4.0, 0, 0, 3, 1.0, 2.0, 0.3, 0.05, 0.42, 0.11), CAM);

    assertEquals(0.42, s.maxAmbiguity());
  }

  @Test
  void decodeReportsNoTargetsWhenTagCountIsZero() {
    CameraSample s = Vision.decode(botpose(1_000_000L, 0, 0, 0, 0, 0, 0, 0, 0), CAM);

    assertFalse(s.hasTarget());
    assertTrue(s.connected(), "the camera is alive, it just sees nothing");
    assertTrue(s.freshFrame());
  }

  @Test
  @DisplayName("a truncated fiducial block leaves ambiguity at 0 rather than reading past the end")
  void decodeSurvivesAMismatchedArrayLength() {
    // Claims 3 tags but only carries data for 1 -- the length guard must skip the scan.
    double[] v = botpose(1_000_000L, 3.0, 4.0, 0, 0, 3, 1.0, 2.0, 0.3, 0.9).value;
    CameraSample s = Vision.decode(new TimestampedDoubleArray(1_000_000L, 1_000_000L, v), CAM);

    assertEquals(3, s.tagCount());
    assertEquals(0.0, s.maxAmbiguity());
  }

  @Test
  void decodeReportsNoTargetsForAShortArray() {
    CameraSample s =
        Vision.decode(new TimestampedDoubleArray(1L, 1L, new double[] {1, 2, 3}), CAM);

    assertFalse(s.hasTarget());
  }
}
