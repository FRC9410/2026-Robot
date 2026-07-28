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

/**
 * Polls every configured Limelight once per loop and reports the pose measurements worth believing.
 *
 * <p>Self-contained on purpose: config, data types, decision logic and NetworkTables plumbing all
 * live here so the class can be dropped into another project as a single file. It holds no
 * reference to a drivetrain -- headings arrive through {@link Config} suppliers and measurements go
 * back out through {@link #getAcceptedMeasurements()} for the caller to apply.
 *
 * <p>Deliberately NOT a {@code SubsystemBase}. {@code SubsystemBase}'s constructor registers with
 * the CommandScheduler and {@code super()} runs before field initializers, so a Vision held as a
 * field of another subsystem registers second and its {@code periodic()} would run *after* its
 * consumer's. An explicit {@link #update()} call makes the ordering guaranteed instead of
 * incidental.
 *
 * <h2>How it works</h2>
 *
 * MegaTag1 solves the full pose from tag geometry alone, but its rotation is poorly conditioned
 * with a single tag. MegaTag2 only solves x/y because we hand it the heading, which is far more
 * accurate -- but its reported rotation is just our own heading echoed back, so it can never
 * correct a wrong gyro. Hence: MegaTag1 once to seed heading, MegaTag2 forever after for position.
 */
public class Vision {

  /** Loops a camera may publish nothing before it is treated as gone. 10 loops is ~200 ms. */
  private static final int STALE_POLL_LIMIT = 10;

  /**
   * Loops of nothing-but-JUMPED verdicts before the pose seed is re-armed. Odometry that has
   * drifted past the jump gate would otherwise reject every correction forever -- see
   * {@link Verdict#JUMPED}.
   */
  private static final int JUMP_LOCKOUT_LIMIT = 50;

  /** MegaTag2: pose solved using the heading we push. No absolute heading of its own. */
  private static final String MT2_ENTRY = "botpose_orb_wpiblue";

  /** MegaTag1: solves heading from tag geometry. The only thing that can seed a cold gyro. */
  private static final String MT1_ENTRY = "botpose_wpiblue";

  /** Below this, distance terms stop being meaningful and start blowing up the trust curve. */
  private static final double MIN_MEANINGFUL_DIST_METERS = 0.5;

  // ===========================================================================================
  // Configuration
  // ===========================================================================================

  /**
   * @param headingDegrees field-relative, blue-origin robot heading for MegaTag2 (0 deg = facing
   *     the red wall). Use the pose estimator's rotation, not the raw gyro: the estimator carries
   *     the reset offset, the raw gyro only matches if it happened to be zeroed at the red wall.
   * @param yawRateDegPerSec angular velocity about world Z; pass {@code () -> 0.0} if the sign
   *     convention is in doubt, the Limelight documents this input as optional
   * @param currentPose current best estimate, used only for the jump gate
   * @param fuseAllCameras when true every accepted camera is reported; when false only the
   *     highest-scoring one is
   */
  public record Config(
      List<CameraConfig> cameras,
      DoubleSupplier headingDegrees,
      DoubleSupplier yawRateDegPerSec,
      Supplier<Pose2d> currentPose,
      FieldBounds bounds,
      AcceptanceParams acceptance,
      StdDevParams stdDevs,
      boolean fuseAllCameras,
      boolean logTelemetry) {

    /** Sensible defaults: standard acceptance and std devs, single best camera, telemetry on. */
    public Config(
        List<CameraConfig> cameras,
        DoubleSupplier headingDegrees,
        DoubleSupplier yawRateDegPerSec,
        Supplier<Pose2d> currentPose,
        FieldBounds bounds) {
      this(
          cameras,
          headingDegrees,
          yawRateDegPerSec,
          currentPose,
          bounds,
          AcceptanceParams.defaults(),
          StdDevParams.defaults(),
          false,
          true);
    }
  }

  /**
   * One Limelight.
   *
   * @param name its NetworkTables table name, e.g. "limelight-left"
   * @param trust relative weight; 1.0 is nominal, below 1.0 de-weights a camera whose mounting is
   *     less certain (a turret-mounted camera is only as good as the turret encoder)
   * @param enabled false skips the camera entirely without removing it from the list
   */
  public record CameraConfig(String name, double trust, boolean enabled) {

    public CameraConfig(String name) {
      this(name, 1.0, true);
    }

    public CameraConfig(String name, double trust) {
      this(name, trust, true);
    }
  }

  /**
   * Field extents used as a sanity gate on vision poses. Tolerance allows a small amount of
   * overhang past the field edge (a real pose can sit slightly outside when the robot is against a
   * wall) without accepting obvious garbage.
   */
  public record FieldBounds(double xMin, double yMin, double xMax, double yMax, double tolerance) {

    /** Bounds anchored at the origin with the default 0.4 m tolerance. */
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

  /**
   * Thresholds deciding whether a {@link CameraSample} is trustworthy enough to hand to the pose
   * estimator. Single-tag solutions get much tighter gates than multi-tag ones: a single tag has no
   * baseline, so its solve is poorly conditioned and its pose can flip.
   *
   * @param maxSingleTagDist max average tag distance, meters, when only one tag is visible
   * @param maxMultiTagDist max average tag distance, meters, with two or more tags
   * @param maxSingleTagAmbiguity max per-tag ambiguity tolerated for a single-tag solution
   * @param minSingleTagArea min average tag area (fraction of image) for a single-tag solution
   * @param maxJumpSingleTag max distance, meters, a single-tag pose may sit from the current
   *     estimate. Multi-tag poses are exempt from this gate entirely.
   */
  public record AcceptanceParams(
      double maxSingleTagDist,
      double maxMultiTagDist,
      double maxSingleTagAmbiguity,
      double minSingleTagArea,
      double maxJumpSingleTag) {

    public static AcceptanceParams defaults() {
      return new AcceptanceParams(4.0, 6.5, 0.3, 0.08, 1.0);
    }
  }

  /**
   * Coefficients for the vision measurement standard deviations handed to the pose estimator.
   *
   * @param baseXy scales the distance^2 / tagCount trust curve, meters
   * @param minXy floor on the xy standard deviation, meters
   * @param maxXy ceiling on the xy standard deviation, meters
   * @param theta heading standard deviation, radians -- see {@link #defaults()}
   */
  public record StdDevParams(double baseXy, double minXy, double maxXy, double theta) {

    /**
     * MegaTag2's reported rotation is the heading we pushed to the Limelight, so trusting it back
     * would be circular -- theta is pinned at a huge value to make the estimator ignore it.
     *
     * <p>Deliberately 9_999_999.0 and not {@link Double#POSITIVE_INFINITY}: the estimator squares
     * these into a measurement covariance and solves for a Kalman gain, and infinity risks
     * propagating NaN through that computation. The magic number is the standard FRC idiom here.
     */
    public static StdDevParams defaults() {
      return new StdDevParams(0.06, 0.05, 4.0, 9_999_999.0);
    }
  }

  // ===========================================================================================
  // Data
  // ===========================================================================================

  /**
   * One camera's view of the world for one loop, decoded from the Limelight botpose array. Pose
   * components are raw doubles rather than a {@code Pose2d} so a rejected sample never builds one.
   *
   * @param connected false when the camera has never published or has gone silent
   * @param freshFrame true only on the loop a new frame actually arrived
   * @param maxAmbiguity worst per-tag ambiguity in the solution; 0 when unavailable
   * @param timestampSeconds latency-corrected FPGA timestamp, seconds
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
    public static CameraSample noTargets(
        String name, boolean connected, boolean freshFrame, double trust) {
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

  /**
   * An accepted vision pose, ready to hand to a pose estimator. Standard deviations travel with the
   * measurement so several cameras with different trust can be applied in the same loop.
   *
   * @param timestampSeconds raw FPGA seconds. Callers must NOT convert this -- {@code Swerve}
   *     already applies {@code Utils.fpgaToCurrentTime} inside its {@code addVisionMeasurement}
   *     override, and converting here too would double-apply the offset.
   */
  public record Measurement(
      String cameraName,
      Pose2d pose,
      double timestampSeconds,
      int tagCount,
      double avgTagDist,
      double xyStdDev,
      double thetaStdDev) {}

  /**
   * Why a sample was or was not used. Published per camera so a bench session can tell "vision is
   * broken" apart from "a threshold is too tight", which are otherwise indistinguishable from the
   * outside.
   */
  public enum Verdict {
    ACCEPT,
    /** No tags in view, or the camera is considered disconnected. */
    NO_TARGET,
    /** No new frame this loop. Normal for a third of loops -- Limelights run slower than 50 Hz. */
    STALE,
    /** Exactly (0,0), which a cold Limelight reports and which sits inside the field bounds. */
    ORIGIN,
    OUT_OF_BOUNDS,
    TOO_FAR,
    /** Single tag whose pose solution is ambiguous enough to flip. */
    AMBIGUOUS,
    /** Single tag too small in frame to localize from. */
    TOO_SMALL,
    /**
     * Single-tag pose further from the current estimate than the robot could have moved. Sustained
     * JUMPED verdicts re-arm the seed rather than rejecting forever, so drifted odometry recovers.
     */
    JUMPED
  }

  // ===========================================================================================
  // State
  // ===========================================================================================

  private final Config config;
  private final List<CameraConfig> cameras;

  private final DoubleArrayEntry[] mt2Entries;
  private final long[] lastFrameTimestamp;
  private final int[] stalePolls;
  private final CameraSample[] samples;
  private final Verdict[] verdicts;

  private final List<Measurement> accepted = new ArrayList<>();
  private final List<Measurement> acceptedView = Collections.unmodifiableList(accepted);

  private CameraSample bestSample = null;
  private int jumpLockoutCounter = 0;

  /** While armed, MegaTag1 is polled for a one-shot pose seed. Cleared once a seed is taken. */
  private boolean seedArmed = true;

  private Pose2d pendingSeed = null;

  public Vision(Config config) {
    this.config = config;
    this.cameras = config.cameras();

    int n = cameras.size();
    mt2Entries = new DoubleArrayEntry[n];
    lastFrameTimestamp = new long[n];
    stalePolls = new int[n];
    samples = new CameraSample[n];
    verdicts = new Verdict[n];

    for (int i = 0; i < n; i++) {
      CameraConfig cam = cameras.get(i);
      // Cached subscriber -- created once per camera, not once per read.
      mt2Entries[i] = LimelightHelpers.getLimelightDoubleArrayEntry(cam.name(), MT2_ENTRY);
      samples[i] = CameraSample.disconnected(cam.name(), cam.trust());
      verdicts[i] = Verdict.NO_TARGET;
    }
  }

  // ===========================================================================================
  // Per-loop pipeline
  // ===========================================================================================

  /** Runs the whole pipeline. Call once per loop, before anything reads the results. */
  public void update() {
    pushRobotOrientation();
    pollCameras();
    selectMeasurements();
    if (seedArmed) {
      pendingSeed = findSeedPose();
    }
    if (config.logTelemetry()) {
      publishTelemetry();
    }
  }

  /**
   * MegaTag2 is computed on the Limelight from the last orientation we sent it, so every camera
   * needs the heading every loop -- not just whichever one happens to be winning. A camera that
   * goes without it publishes a pose built from a stale heading, which is exactly the moment it
   * would be picked up as "best" and believed.
   *
   * <p>Writes are batched with a single flush at the end rather than one flush per camera.
   */
  private void pushRobotOrientation() {
    double heading = config.headingDegrees().getAsDouble();
    double yawRate = config.yawRateDegPerSec().getAsDouble();
    boolean wroteAny = false;

    for (CameraConfig cam : cameras) {
      if (cam.enabled()) {
        LimelightHelpers.SetRobotOrientation_NoFlush(cam.name(), heading, yawRate, 0, 0, 0, 0);
        wroteAny = true;
      }
    }
    if (wroteAny) {
      LimelightHelpers.Flush();
    }
  }

  private void pollCameras() {
    for (int i = 0; i < cameras.size(); i++) {
      CameraConfig cam = cameras.get(i);
      if (!cam.enabled()) {
        samples[i] = CameraSample.disconnected(cam.name(), cam.trust());
        continue;
      }

      TimestampedDoubleArray raw = mt2Entries[i].getAtomic();

      if (raw.value.length == 0) {
        // Camera has never published: not powered, wrong table name, or still booting.
        samples[i] = CameraSample.disconnected(cam.name(), cam.trust());
        continue;
      }

      if (raw.timestamp == lastFrameTimestamp[i]) {
        // No new frame. Limelights publish slower than the 20 ms loop, so this is normal for a
        // few loops -- but a camera that stops entirely leaves its last array in NetworkTables
        // forever, and without this counter it would keep looking valid indefinitely.
        boolean stillConnected = ++stalePolls[i] < STALE_POLL_LIMIT;
        samples[i] = samples[i].asStale(stillConnected);
        continue;
      }

      lastFrameTimestamp[i] = raw.timestamp;
      stalePolls[i] = 0;
      samples[i] = decode(raw, cam);
    }
  }

  /**
   * Decodes a Limelight botpose array. Layout mirrors LimelightHelpers v1.14
   * {@code getBotPoseEstimate} and must be kept in lockstep with it if the vendor file is updated:
   * [0..5] x, y, z, roll, pitch, yaw; [6] latency ms; [7] tagCount; [8] tagSpan; [9] avgTagDist;
   * [10] avgTagArea; then 7 values per fiducial with ambiguity last.
   *
   * <p>Package-private so it can be unit tested against hand-built arrays -- an index error here
   * produces poses that look entirely plausible and are wrong.
   */
  static CameraSample decode(TimestampedDoubleArray raw, CameraConfig cam) {
    double[] v = raw.value;
    if (v.length < 11) {
      return CameraSample.noTargets(cam.name(), true, true, cam.trust());
    }

    int tagCount = (int) v[7];
    if (tagCount <= 0) {
      return CameraSample.noTargets(cam.name(), true, true, cam.trust());
    }

    double latencyMs = v[6];
    double timestampSeconds = raw.timestamp / 1_000_000.0 - latencyMs / 1000.0;

    // Stride the fiducial block for the worst ambiguity rather than materializing one object per
    // tag. Guarded by the same length check LimelightHelpers uses.
    double maxAmbiguity = 0.0;
    if (v.length == 11 + 7 * tagCount) {
      for (int i = 0; i < tagCount; i++) {
        maxAmbiguity = Math.max(maxAmbiguity, v[11 + i * 7 + 6]);
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
        timestampSeconds,
        cam.trust());
  }

  private void selectMeasurements() {
    accepted.clear();
    bestSample = null;

    Pose2d current = config.currentPose().get();
    boolean havePose = !seedArmed;
    double bestScore = Double.NEGATIVE_INFINITY;
    boolean sawTarget = false;
    boolean allJumped = true;

    for (int i = 0; i < samples.length; i++) {
      CameraSample sample = samples[i];
      Verdict verdict = verdict(sample, config.acceptance(), config.bounds(), current, havePose);
      verdicts[i] = verdict;

      if (sample.hasTarget() && sample.freshFrame()) {
        sawTarget = true;
        if (verdict != Verdict.JUMPED) {
          allJumped = false;
        }
      }

      if (verdict != Verdict.ACCEPT) {
        continue;
      }

      double score = score(sample);
      if (score > bestScore) {
        bestScore = score;
        bestSample = sample;
      }
      if (config.fuseAllCameras()) {
        accepted.add(toMeasurement(sample));
      }
    }

    if (!config.fuseAllCameras() && bestSample != null) {
      accepted.add(toMeasurement(bestSample));
    }

    updateJumpLockout(sawTarget && allJumped);
  }

  /**
   * The jump gate protects against garbage poses, but on its own it is a one-way ratchet: if
   * odometry ever drifts further than the gate allows, every future correction looks implausible
   * and is rejected forever. If the cameras have done nothing but disagree for a full second,
   * believe them and re-arm the seed instead.
   */
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

  /**
   * Reads MegaTag1 for a pose that can seed a cold estimate. Requires two or more tags: MegaTag2's
   * rotation is the heading we fed in, so seeding from it can never correct a wrong gyro, and a
   * single-tag MegaTag1 heading is too poorly conditioned to trust for a hard reset.
   *
   * <p>Only called while a seed is armed, so it costs nothing for the rest of the match.
   */
  private Pose2d findSeedPose() {
    Pose2d best = null;
    int bestTagCount = 1;

    for (int i = 0; i < cameras.size(); i++) {
      CameraConfig cam = cameras.get(i);
      // Skip cameras we already know are gone -- their MegaTag1 array is frozen too, and seeding
      // is a hard pose reset, so it is the last place to trust stale data.
      if (!cam.enabled() || !samples[i].connected()) {
        continue;
      }
      double[] v = LimelightHelpers.getLimelightDoubleArrayEntry(cam.name(), MT1_ENTRY).get();
      if (v.length < 11) {
        continue;
      }
      int tagCount = (int) v[7];
      if (tagCount <= bestTagCount) {
        continue;
      }
      if (v[0] == 0.0 && v[1] == 0.0) {
        continue;
      }
      if (!config.bounds().contains(v[0], v[1])) {
        continue;
      }
      best = new Pose2d(v[0], v[1], Rotation2d.fromDegrees(v[5]));
      bestTagCount = tagCount;
    }
    return best;
  }

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
  }

  // ===========================================================================================
  // Decision logic -- pure functions, no state, no I/O. This is the unit tested part.
  // ===========================================================================================

  /**
   * Relative quality of a sample. Higher is better; {@link Double#NEGATIVE_INFINITY} means unusable
   * so an unusable camera can never win a comparison.
   *
   * <p>The multi-tag bonus dominates every other term on purpose. Two distant tags give a far
   * better conditioned solve than one enormous close one, which is exactly the case an area-only
   * comparison gets backwards.
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
   * Whether a sample may be handed to the pose estimator, and why not when it may not.
   *
   * @param current current pose estimate, for the jump gate
   * @param havePose false before the estimate has been seeded, which disables the jump gate
   */
  public static Verdict verdict(
      CameraSample s,
      AcceptanceParams params,
      FieldBounds bounds,
      Pose2d current,
      boolean havePose) {
    if (!s.hasTarget()) {
      return Verdict.NO_TARGET;
    }
    if (!s.freshFrame()) {
      return Verdict.STALE;
    }
    // A cold or absent Limelight reports exactly (0, 0), which is inside the field and would
    // otherwise sail through the bounds check.
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
    if (!multi) {
      if (s.maxAmbiguity() > params.maxSingleTagAmbiguity()) {
        return Verdict.AMBIGUOUS;
      }
      if (s.avgTagArea() < params.minSingleTagArea()) {
        return Verdict.TOO_SMALL;
      }
      // Multi-tag solutions are exempt: they are well conditioned enough to believe even when they
      // disagree with the current estimate, which is what lets drifted odometry get corrected.
      if (havePose
          && current != null
          && Math.hypot(s.x() - current.getX(), s.y() - current.getY())
              > params.maxJumpSingleTag()) {
        return Verdict.JUMPED;
      }
    }
    return Verdict.ACCEPT;
  }

  /** Convenience wrapper over {@link #verdict}. */
  public static boolean isAcceptable(
      CameraSample s,
      AcceptanceParams params,
      FieldBounds bounds,
      Pose2d current,
      boolean havePose) {
    return verdict(s, params, bounds, current, havePose) == Verdict.ACCEPT;
  }

  /**
   * Translational standard deviation for the pose estimator: trust falls off with the square of
   * distance and improves with tag count, which is the standard formulation. A camera configured
   * with lower trust gets a proportionally larger deviation.
   */
  public static double xyStdDev(CameraSample s, StdDevParams params) {
    double dist = Math.max(s.avgTagDist(), MIN_MEANINGFUL_DIST_METERS);
    double raw =
        params.baseXy() * dist * dist / Math.max(s.tagCount(), 1) / Math.max(s.trust(), 1e-3);
    return MathUtil.clamp(raw, params.minXy(), params.maxXy());
  }

  // ===========================================================================================
  // Results
  // ===========================================================================================

  /**
   * Measurements to hand to a pose estimator this loop. Every accepted camera when
   * {@link Config#fuseAllCameras()} is set, otherwise just the best one.
   *
   * <p>The returned list is reused between loops -- read it before the next {@link #update()}.
   */
  public List<Measurement> getAcceptedMeasurements() {
    return acceptedView;
  }

  /** Name of the highest-scoring accepted camera, or "" when none is usable. */
  public String getBestCamera() {
    return bestSample == null ? "" : bestSample.name();
  }

  public Optional<CameraSample> getBestSample() {
    return Optional.ofNullable(bestSample);
  }

  /** Re-arms the one-shot MegaTag1 seed, e.g. when the driver resets the gyro. */
  public void requestSeed() {
    seedArmed = true;
  }

  /**
   * Returns a MegaTag1 seed pose at most once per arming, disarming the seed as it does. Empty
   * until a camera sees two or more tags.
   */
  public Optional<Pose2d> consumeSeedPose() {
    if (!seedArmed || pendingSeed == null) {
      return Optional.empty();
    }
    Pose2d seed = pendingSeed;
    pendingSeed = null;
    seedArmed = false;
    return Optional.of(seed);
  }

  public boolean isSeedPending() {
    return seedArmed;
  }

  /** Latest sample for a camera, or an empty sample if the name is not configured. */
  public CameraSample getSample(String cameraName) {
    for (CameraSample s : samples) {
      if (s.name().equals(cameraName)) {
        return s;
      }
    }
    return CameraSample.disconnected(cameraName, 0.0);
  }

  /** Why the named camera was or was not used this loop. */
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
