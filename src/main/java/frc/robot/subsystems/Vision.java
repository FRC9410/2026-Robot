// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.LimelightHelpers;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// Polls every configured Limelight once per loop and reports the pose measurements worth believing.
//
// Self-contained on purpose: config, data types, decision logic and NetworkTables plumbing all live
// in this one file so it can be dropped into another project as a single unit. It holds no reference
// to a drivetrain -- everything it needs about the robot arrives through Config suppliers, and
// measurements go back out through getAcceptedMeasurements() for the caller to apply.
//
// Deliberately NOT a SubsystemBase. SubsystemBase's constructor registers with the CommandScheduler
// and super() runs before field initializers, so a Vision held as a field of another subsystem
// registers second and its periodic() would run AFTER its consumer's -- meaning the consumer reads
// last loop's data. An explicit update() call makes the ordering guaranteed instead of incidental.
//
// How the two MegaTag modes are used:
//   MegaTag1 solves the full pose from tag geometry alone. Self-contained, but with a single tag the
//     rotation solve is ambiguous and can flip between two plausible answers.
//   MegaTag2 only solves x/y because we hand it the heading. Far more accurate -- but its reported
//     rotation is just our own heading echoed back, so it can never correct a wrong gyro.
// Hence: MegaTag1 to seed heading, MegaTag2 continuously for position.
//
// MegaTag2 is deliberately NOT withheld until that seed lands, even though an unseeded heading makes
// its x/y wrong too. Withholding it is the obvious-looking fix and it is a trap: seedArmed stays true
// until some camera sees two tags at once, so gating on it hands a robot that only ever sees single
// tags a match with no vision whatsoever.
//
// That is a judgement call, not a free win. A wrong heading near the middle of the field can still
// put MegaTag2 somewhere plausible and in bounds, and nothing in here will catch it -- the bounds
// check only reliably rejects errors large enough to throw the pose off the field entirely. What
// makes the trade worth taking is the asymmetry in duration: a bad pose is overwritten within a loop
// of the seed arriving, while gating removes vision for as long as the seed does NOT arrive, which
// can be the entire match.
public class Vision {

  // Loops a camera may publish nothing before it is treated as gone. 10 loops is about 200 ms.
  private static final int STALE_POLL_LIMIT = 10;

  // Loops of nothing-but-JUMPED verdicts before the pose seed is re-armed. Odometry that has
  // drifted past the jump gate would otherwise reject every correction forever. See Verdict.JUMPED.
  private static final int JUMP_LOCKOUT_LIMIT = 50;

  // MegaTag2: pose solved using the heading we push. No absolute heading of its own.
  private static final String MT2_ENTRY = "botpose_orb_wpiblue";

  // MegaTag1: solves heading from tag geometry. The only thing that can seed a cold gyro.
  private static final String MT1_ENTRY = "botpose_wpiblue";

  // Below this, distance terms stop being meaningful and start blowing up the trust curve.
  private static final double MIN_MEANINGFUL_DIST_METERS = 0.5;

  // ===========================================================================================
  // Configuration
  // ===========================================================================================

  // Everything this class needs, with no reference to a specific drivetrain.
  //
  // headingDegrees must be field-relative and blue-origin (0 deg = facing the red wall). Use the
  // pose estimator's rotation, NOT the raw gyro: the estimator carries the reset offset, while the
  // raw gyro only matches if it happened to be zeroed pointing at the red wall.
  //
  // pitch, roll and yaw rate describe what the robot is physically doing. They gate measurements --
  // a tilted or fast-spinning robot produces poses the camera cannot solve correctly.
  //
  // currentPose is the current best estimate, used only for the jump gate.
  //
  // fuseAllCameras true reports every accepted camera; false reports only the highest-scoring one.
  public record Config(
      List<CameraConfig> cameras,
      DoubleSupplier headingDegrees,
      DoubleSupplier pitchDegrees,
      DoubleSupplier rollDegrees,
      DoubleSupplier yawRateDegPerSec,
      Supplier<Pose2d> currentPose,
      FieldBounds bounds,
      AcceptanceParams acceptance,
      StdDevParams stdDevs,
      boolean fuseAllCameras,
      boolean logTelemetry) {

    // Convenience overload: standard acceptance and std devs, single best camera, telemetry on.
    public Config(
        List<CameraConfig> cameras,
        DoubleSupplier headingDegrees,
        DoubleSupplier pitchDegrees,
        DoubleSupplier rollDegrees,
        DoubleSupplier yawRateDegPerSec,
        Supplier<Pose2d> currentPose,
        FieldBounds bounds) {
      this(
          cameras,
          headingDegrees,
          pitchDegrees,
          rollDegrees,
          yawRateDegPerSec,
          currentPose,
          bounds,
          AcceptanceParams.defaults(),
          StdDevParams.defaults(),
          false,
          true);
    }
  }

  // One Limelight.
  //
  // name is its NetworkTables table name, e.g. "limelight-left". This has to match what the device
  // itself is configured as in its web UI -- a mismatch shows up as a permanently absent camera.
  //
  // trust is a relative weight. 1.0 is nominal. Below 1.0 de-weights a camera whose mounting is
  // less certain -- one on a moving mechanism is only as good as that mechanism's encoder, and one
  // that bench data shows disagreeing with its neighbours has earned a lower number.
  //
  // enabled false skips the camera entirely without removing it from the list.
  public record CameraConfig(String name, double trust, boolean enabled) {

    public CameraConfig(String name) {
      this(name, 1.0, true);
    }

    public CameraConfig(String name, double trust) {
      this(name, trust, true);
    }
  }

  // Field extents, used as a sanity gate on vision poses. Tolerance allows a small amount of
  // overhang past the field edge -- a real pose can sit slightly outside when the robot is pressed
  // against a wall -- without accepting obvious garbage.
  public record FieldBounds(double xMin, double yMin, double xMax, double yMax, double tolerance) {

    // Bounds anchored at the origin with the default 0.4 m tolerance.
    public FieldBounds(double xMax, double yMax) {
      this(0.0, 0.0, xMax, yMax, 0.4);
    }

    public boolean contains(double x, double y) {
      return x >= xMin - tolerance
          && x <= xMax + tolerance
          && y >= yMin - tolerance
          && y <= yMax + tolerance;
    }
  }

  // Thresholds deciding whether a sample is trustworthy enough to hand to the pose estimator.
  //
  // Single-tag solutions get much tighter gates than multi-tag ones: a single tag gives the solver
  // no baseline, so the solve is poorly conditioned and the pose can flip outright.
  //
  //   maxSingleTagDist       max average tag distance, meters, with only one tag visible
  //   maxMultiTagDist        max average tag distance, meters, with two or more tags
  //   maxSeedDist            max average tag distance, meters, for a MegaTag1 pose seed. Tighter
  //                          than either limit above on purpose -- a measurement gets averaged away
  //                          by the filter, a seed becomes the pose outright. See isSeedable().
  //   maxSingleTagAmbiguity  max per-tag ambiguity tolerated for a single-tag solution
  //   minSingleTagArea       min average tag area, fraction of image, for a single-tag solution
  //   maxJumpSingleTag       max meters a single-tag pose may sit from the current estimate.
  //                          Multi-tag poses are exempt from this gate entirely.
  //   maxTiltDegrees         max absolute pitch or roll before nothing is believed at all
  //   maxYawRateDegPerSec    max spin rate before nothing is believed at all
  public record AcceptanceParams(
      double maxSingleTagDist,
      double maxMultiTagDist,
      double maxSeedDist,
      double maxSingleTagAmbiguity,
      double minSingleTagArea,
      double maxJumpSingleTag,
      double maxTiltDegrees,
      double maxYawRateDegPerSec) {

    public static AcceptanceParams defaults() {
      return new AcceptanceParams(4.0, 6.5, 4.0, 0.3, 0.08, 1.0, 5.0, 720.0);
    }
  }

  // Coefficients for the vision measurement standard deviations handed to the pose estimator.
  //
  //   baseXy  scales the distance^2 / tagCount trust curve, meters
  //   minXy   floor on the xy standard deviation, meters
  //   maxXy   ceiling on the xy standard deviation, meters
  //   theta   heading standard deviation, radians -- see defaults() below
  public record StdDevParams(double baseXy, double minXy, double maxXy, double theta) {

    // MegaTag2's reported rotation is the heading we pushed to the Limelight, so trusting it back
    // would be circular -- theta is pinned at a huge value to make the estimator ignore it.
    //
    // Deliberately 9_999_999.0 and NOT Double.POSITIVE_INFINITY: the estimator squares these into a
    // measurement covariance and solves for a Kalman gain, and infinity risks propagating NaN
    // through that computation. The magic number is the standard FRC idiom here -- do not "fix" it.
    public static StdDevParams defaults() {
      return new StdDevParams(0.06, 0.05, 4.0, 9_999_999.0);
    }
  }

  // ===========================================================================================
  // Data
  // ===========================================================================================

  // What the robot is physically doing this loop, sampled once and shared by every camera.
  //
  // This is the thing tag statistics cannot tell you. A Limelight solving a pose has no idea the
  // chassis is pitched over a bump or mid-spin; both invalidate its answer, and both are only
  // visible from the gyro.
  public record RobotMotion(double pitchDegrees, double rollDegrees, double yawRateDegPerSec) {

    // A stationary, flat robot. Used by tests and as a safe default.
    public static final RobotMotion LEVEL = new RobotMotion(0.0, 0.0, 0.0);

    // Worst absolute tilt on either axis. Either one being large is disqualifying, so the larger
    // of the two is all that matters.
    public double worstTiltDegrees() {
      return Math.max(Math.abs(pitchDegrees), Math.abs(rollDegrees));
    }
  }

  // One camera's view of the world for one loop, decoded from the Limelight botpose array.
  //
  // Pose components are raw doubles rather than a Pose2d so a rejected sample never builds one.
  //
  //   connected         false when the camera has never published or has gone silent
  //   freshFrame        true only on the loop a new frame actually arrived
  //   maxAmbiguity      worst per-tag ambiguity in the solution; 0 when unavailable
  //   ambiguityKnown    false when the fiducial block could not be read, so maxAmbiguity is a
  //                     default rather than a measurement. Has to be checked before believing that
  //                     number -- the default reads as a perfect score. See Verdict.AMBIGUITY_UNKNOWN.
  //   timestampSeconds  latency-corrected FPGA timestamp, seconds
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
      boolean ambiguityKnown,
      double timestampSeconds,
      double trust) {

    // A camera that is present but currently sees nothing usable.
    //
    // Ambiguity counts as known here: there are no tags whose ambiguity could have gone unread, and
    // flagging it unknown would put a misleading false on the dashboard for every blind camera.
    public static CameraSample noTargets(
        String name, boolean connected, boolean freshFrame, double trust) {
      return new CameraSample(name, connected, freshFrame, 0, 0, 0, 0, 0, 0, 0, 0, true, 0, trust);
    }

    // A camera that has never published, or has gone silent long enough to be considered gone.
    public static CameraSample disconnected(String name, double trust) {
      return noTargets(name, false, false, trust);
    }

    public boolean hasTarget() {
      return connected && tagCount > 0;
    }

    // Two or more tags give the solver a baseline, which is a far better conditioned problem.
    public boolean isMultiTag() {
      return tagCount >= 2;
    }

    // Same sample carried into a loop where no new frame arrived.
    public CameraSample asStale(boolean stillConnected) {
      return new CameraSample(
          name, stillConnected, false, x, y, yawDegrees,
          tagCount, tagSpan, avgTagDist, avgTagArea, maxAmbiguity, ambiguityKnown,
          timestampSeconds, trust);
    }
  }

  // An accepted vision pose, ready to hand to a pose estimator. Standard deviations travel with the
  // measurement so several cameras with different trust can be applied in the same loop.
  //
  // timestampSeconds is RAW FPGA seconds. Callers must NOT convert it -- Swerve already applies
  // Utils.fpgaToCurrentTime inside its addVisionMeasurement override, and converting here too would
  // double-apply the offset and skew every measurement's position in the estimator's history.
  public record Measurement(
      String cameraName,
      Pose2d pose,
      double timestampSeconds,
      int tagCount,
      double avgTagDist,
      double xyStdDev,
      double thetaStdDev) {}

  // Why a sample was or was not used.
  //
  // Published per camera so a bench session can tell "vision is broken" apart from "a threshold is
  // too tight". Those two look identical from the outside and are the most likely source of
  // confusion on a first run.
  public enum Verdict {
    ACCEPT,

    // No tags in view, or the camera is considered disconnected.
    NO_TARGET,

    // No new frame this loop. Normal for a third of loops -- Limelights run slower than 50 Hz.
    STALE,

    // Chassis pitched or rolled too far. The camera is not pointing where the Limelight thinks it
    // is, so its floor projection is wrong no matter how good the tags look.
    TILTED,

    // Spinning fast enough that the frame is smeared and the solve is unreliable.
    SPINNING,

    // Exactly (0,0), which a cold Limelight reports and which sits inside the field bounds.
    ORIGIN,

    OUT_OF_BOUNDS,

    TOO_FAR,

    // Single tag whose pose solution is ambiguous enough to flip.
    AMBIGUOUS,

    // Single tag whose ambiguity could not be read at all, because the fiducial block length did not
    // match the advertised tag count. Unread ambiguity decodes as 0.0 -- a PERFECT score -- so this
    // fails closed rather than letting a malformed array through as the cleanest sighting of the
    // match. Seeing this steadily means the array layout no longer matches decode(), most likely
    // after a Limelight firmware or LimelightHelpers update. It is a "fix the decoder" signal, not
    // a "the tags are bad" one.
    AMBIGUITY_UNKNOWN,

    // Single tag too small in frame to localize from.
    TOO_SMALL,

    // Single-tag pose further from the current estimate than the robot could have moved. Sustained
    // JUMPED verdicts re-arm the seed rather than rejecting forever, so drifted odometry recovers.
    JUMPED
  }

  // ===========================================================================================
  // State
  // ===========================================================================================

  private final Config config;
  private final List<CameraConfig> cameras;

  // One cached NetworkTables subscriber per camera. Created once in the constructor, not per read.
  private final DoubleArrayEntry[] mt2Entries;

  // MegaTag1 subscribers. Cached alongside the MegaTag2 ones, but only READ while a seed is armed,
  // so the "costs nothing for the rest of the match" property still holds.
  private final DoubleArrayEntry[] mt1Entries;

  // Last MegaTag1 NT timestamp actually consumed per camera, so one frame can never seed twice.
  private final long[] lastSeedFrameTimestamp;

  // Last NT timestamp seen per camera, used to detect "no new frame".
  private final long[] lastFrameTimestamp;

  // Consecutive loops each camera has published nothing new.
  private final int[] stalePolls;

  // Latest decoded sample and verdict per camera. Index-aligned with the cameras list.
  private final CameraSample[] samples;
  private final Verdict[] verdicts;

  // Rebuilt each loop. The unmodifiable view is what callers get, so they cannot mutate our list.
  private final List<Measurement> accepted = new ArrayList<>();
  private final List<Measurement> acceptedView = Collections.unmodifiableList(accepted);

  private CameraSample bestSample = null;
  private RobotMotion motion = RobotMotion.LEVEL;

  // Counts consecutive loops where every camera with a fresh view returned JUMPED.
  private int jumpLockoutCounter = 0;

  // While armed, MegaTag1 is polled for a one-shot pose seed. Cleared once a seed is taken.
  // This class is the ONLY owner of this flag -- see consumeSeedPose().
  private boolean seedArmed = true;

  private Pose2d pendingSeed = null;

  public Vision(Config config) {
    this.config = config;
    this.cameras = config.cameras();

    int n = cameras.size();
    mt2Entries = new DoubleArrayEntry[n];
    mt1Entries = new DoubleArrayEntry[n];
    lastSeedFrameTimestamp = new long[n];
    lastFrameTimestamp = new long[n];
    stalePolls = new int[n];
    samples = new CameraSample[n];
    verdicts = new Verdict[n];

    // Set up one cached subscriber per camera and start everything in the disconnected state, so a
    // camera that never publishes reads as absent rather than as a valid pose at the origin.
    for (int i = 0; i < n; i++) {
      CameraConfig cam = cameras.get(i);
      mt2Entries[i] = LimelightHelpers.getLimelightDoubleArrayEntry(cam.name(), MT2_ENTRY);
      mt1Entries[i] = LimelightHelpers.getLimelightDoubleArrayEntry(cam.name(), MT1_ENTRY);
      samples[i] = CameraSample.disconnected(cam.name(), cam.trust());
      verdicts[i] = Verdict.NO_TARGET;
    }
  }

  // ===========================================================================================
  // Per-loop pipeline
  // ===========================================================================================

  // Runs the whole pipeline. Call once per loop, before anything reads the results.
  public void update() {
    // Sample the gyro once and share it. Reading the suppliers repeatedly could hand different
    // cameras slightly different views of the same instant.
    motion = readMotion();

    pushRobotOrientation(motion);
    pollCameras();
    selectMeasurements(motion);

    // MegaTag1 is only read while a seed is wanted, so this costs nothing for the rest of the match.
    if (seedArmed) {
      pendingSeed = findSeedPose();
    }

    if (config.logTelemetry()) {
      publishTelemetry();
    }
  }

  private RobotMotion readMotion() {
    return new RobotMotion(
        config.pitchDegrees().getAsDouble(),
        config.rollDegrees().getAsDouble(),
        config.yawRateDegPerSec().getAsDouble());
  }

  // MegaTag2 is computed ON the Limelight from the last orientation we sent it, so every camera
  // needs the heading every loop -- not just whichever one happens to be winning. A camera that
  // goes without it publishes a pose built from a stale heading, which is exactly the moment it
  // would be picked up as "best" and believed.
  //
  // Writes are batched with a single flush at the end. SetRobotOrientation (the non-NoFlush
  // variant) forces a full NetworkTables flush on every call, which would be three per loop.
  private void pushRobotOrientation(RobotMotion motion) {
    double heading = config.headingDegrees().getAsDouble();
    boolean wroteAny = false;

    for (CameraConfig cam : cameras) {
      if (cam.enabled()) {
        // Pitch and roll are documented by Limelight as unnecessary for MegaTag2, which consumes
        // only yaw. They are sent anyway because they cost nothing, they match the signature's
        // intent, and firmware may start using them.
        LimelightHelpers.SetRobotOrientation_NoFlush(
            cam.name(),
            heading,
            motion.yawRateDegPerSec(),
            motion.pitchDegrees(),
            0,
            motion.rollDegrees(),
            0);
        wroteAny = true;
      }
    }

    if (wroteAny) {
      LimelightHelpers.Flush();
    }
  }

  // Reads each camera's MegaTag2 array and turns it into a sample. Bails out before doing any work
  // in the common cases where there is nothing new to look at.
  private void pollCameras() {
    for (int i = 0; i < cameras.size(); i++) {
      CameraConfig cam = cameras.get(i);

      // A camera switched off in config never contributes and is never read.
      if (!cam.enabled()) {
        samples[i] = CameraSample.disconnected(cam.name(), cam.trust());
        continue;
      }

      TimestampedDoubleArray raw = mt2Entries[i].getAtomic();

      // Zero-length array means the camera has never published: not powered, wrong table name, or
      // still booting.
      if (raw.value.length == 0) {
        samples[i] = CameraSample.disconnected(cam.name(), cam.trust());
        continue;
      }

      // Same NT timestamp as last loop means no new frame. This is NORMAL for a third of loops,
      // since Limelights publish slower than our 20 ms tick -- but a camera that stops entirely
      // leaves its last array in NetworkTables forever, and without this counter it would keep
      // looking perfectly valid indefinitely. Re-adding an old frame would also double-count the
      // same evidence in the Kalman filter, so a stale sample is never accepted either way.
      if (raw.timestamp == lastFrameTimestamp[i]) {
        boolean stillConnected = ++stalePolls[i] < STALE_POLL_LIMIT;
        samples[i] = samples[i].asStale(stillConnected);
        continue;
      }

      // Genuinely new frame: remember it and decode.
      lastFrameTimestamp[i] = raw.timestamp;
      stalePolls[i] = 0;
      samples[i] = decode(raw, cam);
    }
  }

  // Decodes a Limelight botpose array.
  //
  // Layout mirrors LimelightHelpers v1.14 getBotPoseEstimate and must be kept in lockstep with it
  // if the vendor file is ever updated:
  //   [0..5]  x, y, z, roll, pitch, yaw
  //   [6]     latency, milliseconds
  //   [7]     tag count
  //   [8]     tag span
  //   [9]     average tag distance
  //   [10]    average tag area
  //   then 7 values per fiducial: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity
  //
  // Package-private so it can be unit tested against hand-built arrays. An index error here
  // produces poses that look entirely plausible and are wrong, which is the worst failure mode
  // available, so this is the one method in the file with dedicated tests.
  static CameraSample decode(TimestampedDoubleArray raw, CameraConfig cam) {
    double[] v = raw.value;

    // Too short to even hold the summary block -- treat as seeing nothing rather than reading off
    // the end of the array.
    if (v.length < 11) {
      return CameraSample.noTargets(cam.name(), true, true, cam.trust());
    }

    // The camera is alive and publishing, it just has no tags in view.
    int tagCount = (int) v[7];
    if (tagCount <= 0) {
      return CameraSample.noTargets(cam.name(), true, true, cam.trust());
    }

    // Refuse anything non-finite before it can reach a gate. Every threshold downstream is a > or a
    // < comparison, and NaN makes ALL of them false -- so a NaN distance passes the distance check,
    // a NaN area passes the area check, and a NaN ambiguity passes the ambiguity check. They fail
    // open, every one of them. A NaN yaw is worse still: it goes into the pose estimator and poisons
    // the filter permanently instead of costing one bad loop. This is the same hazard StdDevParams
    // avoids by refusing to use infinity, applied to the data coming the other way.
    if (!Double.isFinite(v[0])
        || !Double.isFinite(v[1])
        || !Double.isFinite(v[5])
        || !Double.isFinite(v[6])
        || !Double.isFinite(v[8])
        || !Double.isFinite(v[9])
        || !Double.isFinite(v[10])) {
      return CameraSample.noTargets(cam.name(), true, true, cam.trust());
    }

    // The photons hit the sensor before the data reached us, so back the reported latency out of
    // the NT timestamp. What the estimator needs is when the picture was taken, not when it landed.
    double latencyMs = v[6];
    double timestampSeconds = raw.timestamp / 1_000_000.0 - latencyMs / 1000.0;

    // Stride the fiducial block for the worst ambiguity rather than materializing an object per
    // tag. The length guard is the same one LimelightHelpers uses: if the array does not match the
    // advertised tag count, the block is untrustworthy and is skipped rather than read past.
    //
    // Skipping it leaves maxAmbiguity at 0.0, which reads as a flawless solution -- so whether it
    // was ever measured has to travel with the sample. Without that flag a malformed array sails
    // straight through the single-tag ambiguity gate looking better than any real sighting.
    boolean ambiguityKnown = v.length == 11 + 7 * tagCount;
    double maxAmbiguity = 0.0;
    if (ambiguityKnown) {
      for (int i = 0; i < tagCount; i++) {
        maxAmbiguity = Math.max(maxAmbiguity, v[11 + i * 7 + 6]);
      }

      // A NaN anywhere in the block propagates through Math.max and would then fail open at the
      // gate. Route it into the same "we could not measure this" path as a malformed block rather
      // than inventing a second way to express the same doubt.
      if (!Double.isFinite(maxAmbiguity)) {
        ambiguityKnown = false;
        maxAmbiguity = 0.0;
      }
    }

    return new CameraSample(
        cam.name(),
        true,
        true,
        v[0],
        v[1],
        v[5],
        tagCount,
        v[8],
        v[9],
        v[10],
        maxAmbiguity,
        ambiguityKnown,
        timestampSeconds,
        cam.trust());
  }

  // Runs every sample through the acceptance gates, ranks the survivors, and builds this loop's
  // measurement list.
  private void selectMeasurements(RobotMotion motion) {
    accepted.clear();
    bestSample = null;

    Pose2d current = config.currentPose().get();

    // The jump gate compares against the current estimate, which is only meaningful once the
    // estimate has actually been seeded. Before that, there is nothing to compare to.
    boolean havePose = !seedArmed;

    double bestScore = Double.NEGATIVE_INFINITY;

    // Tracked for the lockout: did any camera have something to say, and was JUMPED the only thing
    // any of them said?
    boolean sawTarget = false;
    boolean allJumped = true;

    for (int i = 0; i < samples.length; i++) {
      CameraSample sample = samples[i];
      Verdict verdict = verdict(sample, config.acceptance(), config.bounds(), current, havePose, motion);
      verdicts[i] = verdict;

      // Only cameras that actually produced a fresh view this loop count towards the lockout. A
      // blind or stale camera is not evidence of anything.
      if (sample.hasTarget() && sample.freshFrame()) {
        sawTarget = true;
        if (verdict != Verdict.JUMPED) {
          allJumped = false;
        }
      }

      if (verdict != Verdict.ACCEPT) {
        continue;
      }

      // Track the best regardless of fusion mode -- getBestCamera() reports it either way.
      double score = score(sample);
      if (score > bestScore) {
        bestScore = score;
        bestSample = sample;
      }

      if (config.fuseAllCameras()) {
        accepted.add(toMeasurement(sample));
      }
    }

    // Single-camera mode: only the winner contributes. Several cameras looking at the SAME tags
    // produce correlated errors, and feeding all of them makes the filter more confident than the
    // evidence warrants, so this is the conservative default.
    if (!config.fuseAllCameras() && bestSample != null) {
      accepted.add(toMeasurement(bestSample));
    }

    updateJumpLockout(sawTarget && allJumped);
  }

  // The jump gate protects against garbage poses, but on its own it is a one-way ratchet: if
  // odometry ever drifts further than the gate allows, every future correction looks implausible
  // and is rejected forever. If the cameras have done nothing but disagree for a full second,
  // believe them and re-arm the seed instead.
  //
  // Re-arming does two things. It sets havePose false, which disables the gate so measurements start
  // flowing again, and it lets the next multi-tag view reseat the pose properly. Consuming that seed
  // then re-enables the gate.
  //
  // Both take effect on the NEXT loop rather than this one: selectMeasurements() reads havePose at
  // its top and only calls this at its end. That is one 20 ms loop of lag on the tail of a 50 loop
  // lockout, so it is left as is rather than restructured -- but do not read the recovery as
  // instantaneous, and do not write a test that expects it to be.
  private void updateJumpLockout(boolean everythingJumped) {
    if (!everythingJumped) {
      jumpLockoutCounter = 0;
      return;
    }
    if (++jumpLockoutCounter >= JUMP_LOCKOUT_LIMIT) {
      jumpLockoutCounter = 0;
      seedArmed = true;
    }
  }

  // Turns an accepted sample into a measurement, building the Pose2d and computing confidence.
  private Measurement toMeasurement(CameraSample s) {
    return new Measurement(
        s.name(),
        new Pose2d(s.x(), s.y(), Rotation2d.fromDegrees(s.yawDegrees())),
        s.timestampSeconds(),
        s.tagCount(),
        s.avgTagDist(),
        xyStdDev(s, config.stdDevs()),
        config.stdDevs().theta());
  }

  // Reads MegaTag1 for a pose that can seed a cold estimate.
  //
  // Requires two or more tags. MegaTag2's rotation is the heading we fed in, so seeding from it can
  // never correct a wrong gyro, and a single-tag MegaTag1 heading is too poorly conditioned to
  // trust for a hard pose reset.
  //
  // The rules themselves live in isSeedable(), which is pure and tested. What stays here is only the
  // NetworkTables plumbing: read, decode, gate, rank. They are TIGHTER than the measurement path's,
  // not looser, and that asymmetry is the point -- a measurement is one vote into a filter that can
  // outvote it a loop later, while a seed becomes the pose outright.
  private Pose2d findSeedPose() {
    Pose2d best = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (int i = 0; i < cameras.size(); i++) {
      CameraConfig cam = cameras.get(i);

      // Skip cameras we already know are gone before spending a read on them.
      if (!cam.enabled() || !samples[i].connected()) {
        continue;
      }

      TimestampedDoubleArray raw = mt1Entries[i].getAtomic();
      if (raw.value.length == 0) {
        continue;
      }

      // MegaTag1 gets its OWN freshness check rather than inheriting MegaTag2's. connected() only
      // proves the MegaTag2 entry has moved recently; nothing actually guarantees the two entries
      // advance in lockstep, and inferring one from the other is the kind of assumption that holds
      // right up until a firmware change quietly breaks it. Re-seeding from a frame already spent
      // would also mean a hard pose reset built on evidence we have used once already.
      if (raw.timestamp == lastSeedFrameTimestamp[i]) {
        continue;
      }
      lastSeedFrameTimestamp[i] = raw.timestamp;

      // The same decoder the measurement path uses. Hand-indexing this array here as well would put
      // a SECOND, untested copy of the layout in the file -- and decode() carries dedicated tests
      // precisely because an index error in it is invisible, permanent, and produces poses that look
      // entirely reasonable. A duplicate of it drifting out of step is that same bug with no test.
      CameraSample s = decode(raw, cam);

      if (!isSeedable(s, config.acceptance(), config.bounds(), motion)) {
        continue;
      }

      // Ranked by the same score the measurement path ranks by, rather than by tag count alone. Tag
      // count gets this backwards whenever the counts differ: error grows with the SQUARE of
      // distance, so a two-tag view up close is a far better reset than a three-tag view across the
      // field, and score() already weighs count, span, area, distance and ambiguity together.
      double sc = score(s);
      if (sc > bestScore) {
        bestScore = sc;
        best = new Pose2d(s.x(), s.y(), Rotation2d.fromDegrees(s.yawDegrees()));
      }
    }

    return best;
  }

  // Per-camera diagnostics. The verdict string is the important one -- it turns "nothing is being
  // accepted" from a mystery into a named reason.
  private void publishTelemetry() {
    for (int i = 0; i < samples.length; i++) {
      CameraSample s = samples[i];
      String prefix = "Vision/" + s.name() + "/";
      SmartDashboard.putBoolean(prefix + "connected", s.connected());
      SmartDashboard.putBoolean(prefix + "freshFrame", s.freshFrame());
      SmartDashboard.putNumber(prefix + "tagCount", s.tagCount());
      SmartDashboard.putNumber(prefix + "avgTagDist", s.avgTagDist());
      SmartDashboard.putNumber(prefix + "avgTagArea", s.avgTagArea());
      SmartDashboard.putNumber(prefix + "maxAmbiguity", s.maxAmbiguity());

      // Read maxAmbiguity above together with this one -- a 0.0 next to a false is not a clean
      // solution, it is an unread fiducial block, and it means decode() no longer matches what the
      // Limelight publishes.
      SmartDashboard.putBoolean(prefix + "ambiguityKnown", s.ambiguityKnown());
      SmartDashboard.putNumber(prefix + "score", score(s));
      SmartDashboard.putString(prefix + "verdict", verdicts[i].name());
    }

    for (Measurement m : accepted) {
      SmartDashboard.putNumber("Vision/" + m.cameraName() + "/xyStdDev", m.xyStdDev());
    }

    SmartDashboard.putString("Vision/bestCamera", getBestCamera());
    SmartDashboard.putNumber("Vision/acceptedCount", accepted.size());
    SmartDashboard.putBoolean("Vision/seedArmed", seedArmed);
    SmartDashboard.putNumber("Vision/jumpLockout", jumpLockoutCounter);

    // Robot motion, so a bench session can see why TILTED or SPINNING is firing.
    SmartDashboard.putNumber("Vision/tiltDegrees", motion.worstTiltDegrees());
    SmartDashboard.putNumber("Vision/yawRateDegPerSec", motion.yawRateDegPerSec());
  }

  // ===========================================================================================
  // Decision logic -- pure functions, no state, no I/O. This is the unit tested part.
  // ===========================================================================================

  // Relative quality of a sample. Higher is better. NEGATIVE_INFINITY means unusable, so an
  // unusable camera can never win a comparison.
  //
  // The multi-tag bonus dominates every other term on purpose. Two distant tags give a far better
  // conditioned solve than one enormous close one, which is exactly the case an area-only
  // comparison gets backwards.
  public static double score(CameraSample s) {
    if (!s.hasTarget()) {
      return Double.NEGATIVE_INFINITY;
    }

    double dist = Math.max(s.avgTagDist(), MIN_MEANINGFUL_DIST_METERS);

    double raw =
        // Having a baseline at all is worth more than everything else combined.
        (s.isMultiTag() ? 100.0 : 0.0)
            // More tags still help past two, with diminishing returns past four.
            + 15.0 * Math.min(s.tagCount(), 4)
            // Tags spread wide across the image constrain the solve better than clustered ones.
            + 8.0 * s.tagSpan()
            // Bigger in frame is better, square-rooted because the benefit tails off.
            + 20.0 * Math.sqrt(Math.max(s.avgTagArea(), 0.0))
            // Distance hurts.
            - 6.0 * dist
            // The worst tag drags the whole solution down with it.
            - 30.0 * s.maxAmbiguity();

    // Trust has to move the score the same direction whatever the sign of raw. raw goes NEGATIVE for
    // a marginal single-tag sample that still passed every gate -- one tag at the 4 m distance cap,
    // the 0.08 area floor and the 0.3 ambiguity cap scores about -12 -- and multiplying a negative
    // by a trust below 1.0 raises it, handing the win to whichever camera we believe least. Dividing
    // on that branch keeps lower trust ranking lower everywhere. Same zero guard xyStdDev() uses.
    return raw >= 0.0 ? raw * s.trust() : raw / Math.max(s.trust(), 1e-3);
  }

  // Whether a sample may be handed to the pose estimator, and why not when it may not.
  //
  // Order matters. Cheap and universal reasons come first, then whole-robot state, then per-sample
  // geometry -- so the reported verdict is the most fundamental reason rather than an incidental
  // one. havePose false disables the jump gate, which is the case before the estimate is seeded.
  public static Verdict verdict(
      CameraSample s,
      AcceptanceParams params,
      FieldBounds bounds,
      Pose2d current,
      boolean havePose,
      RobotMotion motion) {

    // Nothing to evaluate.
    if (!s.hasTarget()) {
      return Verdict.NO_TARGET;
    }

    // Already-seen frame. Accepting it again would feed the filter the same evidence twice.
    if (!s.freshFrame()) {
      return Verdict.STALE;
    }

    // Whole-robot state next. If the chassis is tilted or spinning hard, no camera's answer is
    // trustworthy, so this overrides anything the tag statistics might suggest. This is the one
    // class of error tag data cannot reveal on its own -- a pose from a pitched robot looks
    // perfectly healthy from the inside.
    if (motion != null) {
      if (motion.worstTiltDegrees() > params.maxTiltDegrees()) {
        return Verdict.TILTED;
      }
      if (Math.abs(motion.yawRateDegPerSec()) > params.maxYawRateDegPerSec()) {
        return Verdict.SPINNING;
      }
    }

    // A cold or absent Limelight reports exactly (0,0), which is inside the field and would
    // otherwise sail straight through the bounds check below.
    if (s.x() == 0.0 && s.y() == 0.0) {
      return Verdict.ORIGIN;
    }

    if (!bounds.contains(s.x(), s.y())) {
      return Verdict.OUT_OF_BOUNDS;
    }

    boolean multi = s.isMultiTag();

    if (s.avgTagDist() > (multi ? params.maxMultiTagDist() : params.maxSingleTagDist())) {
      return Verdict.TOO_FAR;
    }

    // Everything below applies to single-tag solutions only. Multi-tag has a baseline, so it is not
    // vulnerable to the flip that makes these checks necessary.
    if (!multi) {
      // "Could not measure it" has to be caught BEFORE the threshold test, because an unmeasured
      // ambiguity is 0.0 and would clear that threshold more cleanly than any real tag. With one tag
      // this check is the only thing standing between us and a pose that has silently flipped, so
      // the unknown case is rejected rather than assumed good.
      if (!s.ambiguityKnown()) {
        return Verdict.AMBIGUITY_UNKNOWN;
      }
      if (s.maxAmbiguity() > params.maxSingleTagAmbiguity()) {
        return Verdict.AMBIGUOUS;
      }
      if (s.avgTagArea() < params.minSingleTagArea()) {
        return Verdict.TOO_SMALL;
      }

      // Multi-tag is deliberately exempt from this gate: it is well conditioned enough to believe
      // even when it disagrees with the current estimate, which is what lets drifted odometry get
      // corrected instead of permanently locked out.
      if (havePose
          && current != null
          && Math.hypot(s.x() - current.getX(), s.y() - current.getY())
              > params.maxJumpSingleTag()) {
        return Verdict.JUMPED;
      }
    }

    return Verdict.ACCEPT;
  }

  // Convenience wrapper over verdict() for callers that only care yes or no.
  public static boolean isAcceptable(
      CameraSample s,
      AcceptanceParams params,
      FieldBounds bounds,
      Pose2d current,
      boolean havePose,
      RobotMotion motion) {
    return verdict(s, params, bounds, current, havePose, motion) == Verdict.ACCEPT;
  }

  // Whether a MegaTag1 sample is good enough to HARD RESET the pose from.
  //
  // Kept separate from verdict() because it answers a different question with a different cost of
  // being wrong. A measurement that slips through is one vote into a filter that can outvote it a
  // loop later; a seed that slips through simply becomes the pose. So this is deliberately stricter
  // -- and deliberately has no equivalent of the jump gate, because reseating a drifted estimate is
  // the whole point of a seed. Disagreeing with the current pose is not evidence against a seed.
  //
  // Pure and public so the seed rules can actually be tested. findSeedPose() cannot be: it needs a
  // live NetworkTables to reach a Limelight, which is exactly why the rules do not live inside it.
  public static boolean isSeedable(
      CameraSample s, AcceptanceParams params, FieldBounds bounds, RobotMotion motion) {

    // Two or more tags, always. A single-tag MegaTag1 rotation can flip between two plausible
    // answers, and a flipped heading applied as a hard reset is worse than never seeding at all --
    // every MegaTag2 pose from then on is built on top of it.
    if (!s.hasTarget() || !s.isMultiTag()) {
      return false;
    }

    // Whole-robot state, same reasoning and same order as verdict(). A pitched or spinning chassis
    // invalidates MegaTag1 exactly as it invalidates MegaTag2, and neither shows up anywhere in the
    // tag statistics. Refusing costs nothing: the seed stays armed and simply waits for a better
    // moment, which on a robot that is mid-bump is milliseconds away.
    if (motion != null) {
      if (motion.worstTiltDegrees() > params.maxTiltDegrees()) {
        return false;
      }
      if (Math.abs(motion.yawRateDegPerSec()) > params.maxYawRateDegPerSec()) {
        return false;
      }
    }

    // Tighter than either measurement distance limit -- see AcceptanceParams.maxSeedDist.
    if (s.avgTagDist() > params.maxSeedDist()) {
      return false;
    }

    // The same two sanity checks the measurement path uses: a cold Limelight reports exactly (0,0),
    // which reads as a perfectly valid in-bounds pose, and anything off the field is garbage.
    if (s.x() == 0.0 && s.y() == 0.0) {
      return false;
    }
    return bounds.contains(s.x(), s.y());
  }

  // Translational standard deviation for the pose estimator.
  //
  // Trust falls off with the SQUARE of distance because a fixed pixel error is an angular error --
  // the same angle covers more ground the further away the tag is. It improves with tag count
  // because independent measurements average down. A camera configured with lower trust gets a
  // proportionally larger deviation.
  public static double xyStdDev(CameraSample s, StdDevParams params) {
    double dist = Math.max(s.avgTagDist(), MIN_MEANINGFUL_DIST_METERS);

    double raw =
        params.baseXy() * dist * dist / Math.max(s.tagCount(), 1) / Math.max(s.trust(), 1e-3);

    return MathUtil.clamp(raw, params.minXy(), params.maxXy());
  }

  // ===========================================================================================
  // Results
  // ===========================================================================================

  // Measurements to hand to a pose estimator this loop. Every accepted camera when fuseAllCameras
  // is set, otherwise just the best one.
  //
  // The returned list is reused between loops -- read it before the next update().
  public List<Measurement> getAcceptedMeasurements() {
    return acceptedView;
  }

  // Name of the highest-scoring accepted camera, or "" when none is usable.
  public String getBestCamera() {
    return bestSample == null ? "" : bestSample.name();
  }

  public Optional<CameraSample> getBestSample() {
    return Optional.ofNullable(bestSample);
  }

  // Re-arms the one-shot MegaTag1 seed, e.g. when the driver resets the gyro.
  public void requestSeed() {
    seedArmed = true;
  }

  // Returns a MegaTag1 seed pose at most once per arming, disarming the seed as it does. Empty
  // until a camera sees two or more tags.
  //
  // This class is the single owner of whether a seed is wanted -- requestSeed() and the jump
  // lockout both arm it, and only this method disarms it. Callers should invoke this every loop and
  // act on whatever comes back. Do NOT keep a separate "already seeded" flag alongside it: a
  // caller-side flag cannot see the lockout re-arming, and will silently suppress the recovery.
  public Optional<Pose2d> consumeSeedPose() {
    if (!seedArmed || pendingSeed == null) {
      return Optional.empty();
    }
    Pose2d seed = pendingSeed;
    pendingSeed = null;
    seedArmed = false;
    return Optional.of(seed);
  }

  // Whether a seed is currently WANTED -- not whether one is ready to hand out. Armed means
  // requestSeed() or the jump lockout has asked for a reseat and consumeSeedPose() has not yet
  // delivered one, so this stays true for as long as it takes a camera to see two tags. Named to
  // match the seedArmed field and the "Vision/seedArmed" telemetry key.
  public boolean isSeedArmed() {
    return seedArmed;
  }

  // What the gyro said this loop. Useful for logging alongside a rejected measurement.
  public RobotMotion getMotion() {
    return motion;
  }

  // Latest sample for a camera, or an empty sample if the name is not configured.
  public CameraSample getSample(String cameraName) {
    for (CameraSample s : samples) {
      if (s.name().equals(cameraName)) {
        return s;
      }
    }
    return CameraSample.disconnected(cameraName, 0.0);
  }

  // Why the named camera was or was not used this loop.
  public Verdict getVerdict(String cameraName) {
    for (int i = 0; i < samples.length; i++) {
      if (samples[i].name().equals(cameraName)) {
        return verdicts[i];
      }
    }
    return Verdict.NO_TARGET;
  }

  public boolean isConnected(String cameraName) {
    return getSample(cameraName).connected();
  }

  public boolean hasTarget(String cameraName) {
    return getSample(cameraName).hasTarget();
  }
}
