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
import frc.robot.subsystems.Vision.RobotMotion;
import frc.robot.subsystems.Vision.StdDevParams;
import frc.robot.subsystems.Vision.Verdict;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

// No HAL, no NetworkTables -- everything under test is a pure function, so these run on a laptop in
// about two seconds. What they prove is that the code does what was intended; they cannot prove the
// thresholds are right for this robot. That needs bench data.
class VisionTest {

  private static final AcceptanceParams PARAMS = AcceptanceParams.defaults();
  private static final StdDevParams STD_DEVS = StdDevParams.defaults();
  private static final FieldBounds BOUNDS = new FieldBounds(0.0, 0.0, 17.5, 8.0, 0.4);

  // A fresh, connected sample sitting in the middle of the field.
  private static CameraSample sample(
      String name, int tagCount, double area, double dist, double ambiguity) {
    return sample(name, tagCount, area, dist, ambiguity, 8.0, 4.0, 1.0);
  }

  // Full control over position and trust, for the bounds and weighting tests.
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

  // Default case: flat, still robot with no seeded pose, so the jump gate is inactive.
  private static Verdict verdictOf(CameraSample s) {
    return Vision.verdict(s, PARAMS, BOUNDS, null, false, RobotMotion.LEVEL);
  }

  // -------------------------------------------------------------------------
  // score -- relative ranking between cameras
  // -------------------------------------------------------------------------

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

    // Even the worst real sighting must outrank having nothing at all.
    assertTrue(Vision.score(worst) > Vision.score(blind));
  }

  @Test
  void ambiguityDragsScoreDown() {
    assertTrue(
        Vision.score(sample("l", 1, 0.2, 2.0, 0.0)) > Vision.score(sample("r", 1, 0.2, 2.0, 0.5)));
  }

  @Test
  void closerIsBetterAllElseEqual() {
    assertTrue(
        Vision.score(sample("l", 2, 0.2, 2.0, 0.0)) > Vision.score(sample("r", 2, 0.2, 5.0, 0.0)));
  }

  @Test
  void identicalSamplesTieBreakByTrust() {
    CameraSample trusted = sample("left", 2, 0.2, 2.0, 0.0, 8.0, 4.0, 1.0);
    CameraSample doubted = sample("turret", 2, 0.2, 2.0, 0.0, 8.0, 4.0, 0.8);

    assertTrue(Vision.score(trusted) > Vision.score(doubted));
  }

  // -------------------------------------------------------------------------
  // verdict -- the acceptance gates
  // -------------------------------------------------------------------------

  @Test
  void staleFrameIsRejectedEvenWithGoodData() {
    CameraSample good = sample("left", 2, 0.3, 2.0, 0.0);

    assertEquals(Verdict.ACCEPT, verdictOf(good));

    // Same data, no new frame. Accepting it again would double-count the same evidence.
    assertEquals(Verdict.STALE, verdictOf(good.asStale(true)));
  }

  @Test
  void disconnectedCameraIsRejected() {
    assertEquals(Verdict.NO_TARGET, verdictOf(CameraSample.disconnected("left", 1.0)));
  }

  @Test
  @DisplayName("the (0,0) default pose from a cold Limelight is rejected despite being in bounds")
  void originPoseIsRejected() {
    // The point of this test: bounds checking alone would let it through.
    assertTrue(BOUNDS.contains(0.0, 0.0));
    assertEquals(Verdict.ORIGIN, verdictOf(sample("left", 2, 0.3, 2.0, 0.0, 0.0, 0.0, 1.0)));
  }

  @Test
  void slightlyOutsideFieldIsAcceptedButFarOutsideIsNot() {
    // 0.3 m outside is within the 0.4 m tolerance -- a real pose when pressed against a wall.
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

  // -------------------------------------------------------------------------
  // Motion gates -- the tilt and spin checks
  //
  // These cover the failure mode tag statistics cannot see. A pose solved while the chassis is
  // pitched over a bump looks perfectly healthy from the inside: good tag count, good area, low
  // ambiguity, in bounds. Only the gyro knows it is wrong.
  // -------------------------------------------------------------------------

  @Test
  @DisplayName("a pitched robot is rejected even when the tags themselves look perfect")
  void pitchBeyondLimitIsRejected() {
    // Deliberately the best possible sighting, so the ONLY reason to reject is the tilt.
    CameraSample perfect = sample("left", 4, 0.5, 1.5, 0.0);
    RobotMotion pitched = new RobotMotion(12.0, 0.0, 0.0);

    assertEquals(Verdict.ACCEPT, Vision.verdict(perfect, PARAMS, BOUNDS, null, false, RobotMotion.LEVEL));
    assertEquals(Verdict.TILTED, Vision.verdict(perfect, PARAMS, BOUNDS, null, false, pitched));
  }

  @Test
  void rollBeyondLimitIsRejectedToo() {
    CameraSample perfect = sample("left", 4, 0.5, 1.5, 0.0);
    RobotMotion rolled = new RobotMotion(0.0, -9.0, 0.0);

    // Negative roll must be caught as well -- the check is on absolute value.
    assertEquals(Verdict.TILTED, Vision.verdict(perfect, PARAMS, BOUNDS, null, false, rolled));
  }

  @Test
  void smallTiltIsStillAccepted() {
    CameraSample good = sample("left", 2, 0.3, 2.0, 0.0);

    // 3 degrees is inside the 5 degree default -- normal chassis flex and carpet, not a bump.
    RobotMotion slight = new RobotMotion(3.0, 2.0, 0.0);

    assertEquals(Verdict.ACCEPT, Vision.verdict(good, PARAMS, BOUNDS, null, false, slight));
  }

  @Test
  @DisplayName("worst tilt wins -- one bad axis is disqualifying even if the other is flat")
  void worstTiltIsTheOneThatCounts() {
    assertEquals(9.0, new RobotMotion(0.0, -9.0, 0.0).worstTiltDegrees());
    assertEquals(12.0, new RobotMotion(12.0, 3.0, 0.0).worstTiltDegrees());
    assertEquals(0.0, RobotMotion.LEVEL.worstTiltDegrees());
  }

  @Test
  void fastSpinIsRejected() {
    CameraSample good = sample("left", 2, 0.3, 2.0, 0.0);
    RobotMotion spinning = new RobotMotion(0.0, 0.0, 900.0);
    RobotMotion spinningOtherWay = new RobotMotion(0.0, 0.0, -900.0);

    // Past the 720 deg/s default in either direction.
    assertEquals(Verdict.SPINNING, Vision.verdict(good, PARAMS, BOUNDS, null, false, spinning));
    assertEquals(
        Verdict.SPINNING, Vision.verdict(good, PARAMS, BOUNDS, null, false, spinningOtherWay));
  }

  @Test
  void normalTurningRateIsStillAccepted() {
    CameraSample good = sample("left", 2, 0.3, 2.0, 0.0);

    // A brisk but ordinary turn.
    RobotMotion turning = new RobotMotion(0.0, 0.0, 200.0);

    assertEquals(Verdict.ACCEPT, Vision.verdict(good, PARAMS, BOUNDS, null, false, turning));
  }

  @Test
  @DisplayName("tilt is reported ahead of pose problems, so the verdict names the root cause")
  void motionGatesAreCheckedBeforePoseGates() {
    // This sample is BOTH out of bounds and taken while tilted. The tilt is the more fundamental
    // reason, so that is what should be reported -- fixing the bounds would not help.
    CameraSample outOfBounds = sample("left", 2, 0.3, 2.0, 0.0, -5.0, 4.0, 1.0);
    RobotMotion pitched = new RobotMotion(20.0, 0.0, 0.0);

    assertEquals(Verdict.TILTED, Vision.verdict(outOfBounds, PARAMS, BOUNDS, null, false, pitched));
  }

  @Test
  @DisplayName("no-target and stale still win over the motion gates -- they are cheaper and prior")
  void emptySamplesAreReportedBeforeMotionGates() {
    RobotMotion pitched = new RobotMotion(20.0, 0.0, 0.0);
    CameraSample blind = CameraSample.noTargets("left", true, true, 1.0);
    CameraSample stale = sample("left", 2, 0.3, 2.0, 0.0).asStale(true);

    assertEquals(Verdict.NO_TARGET, Vision.verdict(blind, PARAMS, BOUNDS, null, false, pitched));
    assertEquals(Verdict.STALE, Vision.verdict(stale, PARAMS, BOUNDS, null, false, pitched));
  }

  // -------------------------------------------------------------------------
  // The jump gate and its escape hatch
  // -------------------------------------------------------------------------

  @Test
  void jumpGateRejectsImplausibleSingleTagPoses() {
    Pose2d current = new Pose2d(8.0, 4.0, Rotation2d.kZero);

    // 5 m from the current estimate -- further than the robot could have moved in one loop.
    CameraSample farAway = sample("left", 1, 0.3, 2.0, 0.0, 13.0, 4.0, 1.0);

    assertEquals(
        Verdict.JUMPED, Vision.verdict(farAway, PARAMS, BOUNDS, current, true, RobotMotion.LEVEL));
  }

  @Test
  @DisplayName("multi-tag is exempt from the jump gate, so drifted odometry can still recover")
  void multiTagIsExemptFromTheJumpGate() {
    Pose2d current = new Pose2d(8.0, 4.0, Rotation2d.kZero);

    // Same 5 m disagreement, but two tags. Well conditioned enough to believe over the estimate --
    // without this exemption, drifted odometry would reject every correction forever.
    CameraSample farAway = sample("left", 2, 0.3, 2.0, 0.0, 13.0, 4.0, 1.0);

    assertEquals(
        Verdict.ACCEPT, Vision.verdict(farAway, PARAMS, BOUNDS, current, true, RobotMotion.LEVEL));
  }

  @Test
  @DisplayName("jump gate is disabled before the estimate is seeded")
  void jumpGateIgnoredWhenPoseNotYetSeeded() {
    Pose2d current = new Pose2d(8.0, 4.0, Rotation2d.kZero);
    CameraSample farAway = sample("left", 1, 0.3, 2.0, 0.0, 13.0, 4.0, 1.0);

    // havePose true gates it, false does not -- there is nothing to compare against yet.
    assertEquals(
        Verdict.JUMPED, Vision.verdict(farAway, PARAMS, BOUNDS, current, true, RobotMotion.LEVEL));
    assertEquals(
        Verdict.ACCEPT, Vision.verdict(farAway, PARAMS, BOUNDS, current, false, RobotMotion.LEVEL));
  }

  // -------------------------------------------------------------------------
  // xyStdDev -- the confidence handed to the pose estimator
  // -------------------------------------------------------------------------

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
    // Absurdly good and absurdly bad inputs must not produce absurd confidences.
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

  // -------------------------------------------------------------------------
  // decode -- the botpose array layout
  //
  // The highest-value tests in the file. An index error here produces poses that look entirely
  // plausible and are wrong, forever, silently. The layout is pinned against LimelightHelpers v1.14.
  // -------------------------------------------------------------------------

  private static final CameraConfig CAM = new CameraConfig("limelight-left", 0.8);

  // Builds a botpose array by hand: 11 summary values, then 7 per fiducial. Every value is distinct
  // so a swapped index shows up as a failed assertion rather than a coincidental pass.
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

    // Summary block.
    v[0] = x;
    v[1] = y;
    v[2] = 0.3; // z -- unused by us
    v[3] = 1.0; // roll -- unused
    v[4] = 2.0; // pitch -- unused, and deliberately near yaw to catch an off-by-one
    v[5] = yawDeg;
    v[6] = latencyMs;
    v[7] = tagCount;
    v[8] = tagSpan;
    v[9] = avgDist;
    v[10] = avgArea;

    // Fiducial block, ambiguity last in each group of seven.
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
        Vision.decode(
            botpose(5_000_000L, 3.25, 6.5, 42.0, 20.0, 2, 1.75, 2.5, 0.42, 0.1, 0.05), CAM);

    assertEquals("limelight-left", s.name());
    assertTrue(s.connected());
    assertTrue(s.freshFrame());
    assertEquals(3.25, s.x());
    assertEquals(6.5, s.y());
    assertEquals(42.0, s.yawDegrees(), "yaw is index 5, not 3 or 4");
    assertEquals(2, s.tagCount());
    assertEquals(1.75, s.tagSpan());
    assertEquals(2.5, s.avgTagDist());
    assertEquals(0.42, s.avgTagArea());
    assertEquals(0.8, s.trust(), "trust comes from config, not the array");
  }

  @Test
  @DisplayName("timestamp is NT micros converted to seconds, minus the reported latency")
  void decodeSubtractsLatencyFromTheTimestamp() {
    CameraSample s =
        Vision.decode(botpose(5_000_000L, 3.0, 4.0, 0, 20.0, 1, 0, 2.0, 0.3, 0.0), CAM);

    // 5 s of NT time, 20 ms of latency: the picture was taken at 4.98 s.
    assertEquals(5.0 - 0.020, s.timestampSeconds(), 1e-9);
  }

  @Test
  void decodeTakesTheWorstAmbiguityAcrossTags() {
    CameraSample s =
        Vision.decode(
            botpose(1_000_000L, 3.0, 4.0, 0, 0, 3, 1.0, 2.0, 0.3, 0.05, 0.42, 0.11), CAM);

    // The middle tag is the bad one, so it must not be the first or last that gets picked up.
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
    // Claims 3 tags but only carries data for 1. The length guard must skip the scan entirely
    // instead of walking off the array.
    double[] v = botpose(1_000_000L, 3.0, 4.0, 0, 0, 3, 1.0, 2.0, 0.3, 0.9).value;
    CameraSample s = Vision.decode(new TimestampedDoubleArray(1_000_000L, 1_000_000L, v), CAM);

    assertEquals(3, s.tagCount());
    assertEquals(0.0, s.maxAmbiguity());
  }

  @Test
  void decodeReportsNoTargetsForAShortArray() {
    // Too short to hold even the summary block.
    CameraSample s = Vision.decode(new TimestampedDoubleArray(1L, 1L, new double[] {1, 2, 3}), CAM);

    assertFalse(s.hasTarget());
  }
}
