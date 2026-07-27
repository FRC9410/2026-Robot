// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.CameraSample;
import frc.robot.subsystems.vision.VisionConfig;
import frc.robot.subsystems.vision.VisionConfig.CameraConfig;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.subsystems.vision.VisionScoring;
import frc.robot.utils.LimelightHelpers;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * Polls every configured Limelight once per loop and reports the pose measurements worth believing.
 *
 * <p>Deliberately NOT a {@code SubsystemBase}. {@code SubsystemBase}'s constructor registers with
 * the CommandScheduler, and {@code super()} runs before field initializers -- so a Vision held as a
 * field of another subsystem registers second and its {@code periodic()} runs *after* its consumer's.
 * An explicit {@link #update()} call makes the ordering guaranteed instead of incidental, and keeps
 * this class free of the scheduler so it can move into the robot library as-is.
 *
 * <p>This class never touches a drivetrain. Headings arrive through {@link VisionConfig} suppliers
 * and measurements go back out through {@link #getAcceptedMeasurements()} for the caller to apply.
 */
public class Vision {

  /** Loops a camera may publish nothing before it is treated as gone. 10 loops is ~200 ms. */
  private static final int STALE_POLL_LIMIT = 10;

  /** MegaTag2: pose solved using the heading we push. No absolute heading of its own. */
  private static final String MT2_ENTRY = "botpose_orb_wpiblue";

  /** MegaTag1: solves heading from tag geometry. The only thing that can seed a cold gyro. */
  private static final String MT1_ENTRY = "botpose_wpiblue";

  private final VisionConfig config;
  private final List<CameraConfig> cameras;

  private final DoubleArrayEntry[] mt2Entries;
  private final long[] lastFrameTimestamp;
  private final int[] stalePolls;
  private final CameraSample[] samples;

  private final List<VisionMeasurement> accepted = new ArrayList<>();
  private final List<VisionMeasurement> acceptedView = Collections.unmodifiableList(accepted);

  private CameraSample bestSample = null;

  /** While armed, MegaTag1 is polled for a one-shot pose seed. Cleared once a seed is taken. */
  private boolean seedArmed = true;

  private Pose2d pendingSeed = null;

  public Vision(VisionConfig config) {
    this.config = config;
    this.cameras = config.cameras();

    int n = cameras.size();
    mt2Entries = new DoubleArrayEntry[n];
    lastFrameTimestamp = new long[n];
    stalePolls = new int[n];
    samples = new CameraSample[n];

    for (int i = 0; i < n; i++) {
      CameraConfig cam = cameras.get(i);
      // Cached subscriber -- created once per camera, not once per read.
      mt2Entries[i] = LimelightHelpers.getLimelightDoubleArrayEntry(cam.name(), MT2_ENTRY);
      samples[i] = CameraSample.disconnected(cam.name(), cam.trust());
    }
  }

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
   */
  private static CameraSample decode(TimestampedDoubleArray raw, CameraConfig cam) {
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

    for (CameraSample sample : samples) {
      if (!VisionScoring.isAcceptable(sample, config.acceptance(), config.bounds(), current, havePose)) {
        continue;
      }
      double score = VisionScoring.score(sample);
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
  }

  private VisionMeasurement toMeasurement(CameraSample s) {
    return new VisionMeasurement(
        s.name(),
        new Pose2d(s.x(), s.y(), Rotation2d.fromDegrees(s.yawDegrees())),
        s.timestampSeconds(),
        s.tagCount(),
        s.avgTagDist(),
        VisionScoring.xyStdDev(s, config.stdDevs()),
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

    for (CameraConfig cam : cameras) {
      if (!cam.enabled()) {
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
    for (CameraSample s : samples) {
      String prefix = "Vision/" + s.name() + "/";
      SmartDashboard.putBoolean(prefix + "connected", s.connected());
      SmartDashboard.putBoolean(prefix + "freshFrame", s.freshFrame());
      SmartDashboard.putNumber(prefix + "tagCount", s.tagCount());
      SmartDashboard.putNumber(prefix + "avgTagDist", s.avgTagDist());
      SmartDashboard.putNumber(prefix + "avgTagArea", s.avgTagArea());
      SmartDashboard.putNumber(prefix + "maxAmbiguity", s.maxAmbiguity());
      SmartDashboard.putNumber(prefix + "score", VisionScoring.score(s));
    }
    for (VisionMeasurement m : accepted) {
      SmartDashboard.putNumber("Vision/" + m.cameraName() + "/xyStdDev", m.xyStdDev());
    }
    SmartDashboard.putString("Vision/bestCamera", getBestCamera());
    SmartDashboard.putNumber("Vision/acceptedCount", accepted.size());
    SmartDashboard.putBoolean("Vision/seedArmed", seedArmed);
  }

  // --- results -------------------------------------------------------------

  /**
   * Measurements to hand to a pose estimator this loop. Every accepted camera when
   * {@link VisionConfig#fuseAllCameras()} is set, otherwise just the best one.
   *
   * <p>The returned list is reused between loops -- read it before the next {@link #update()}.
   */
  public List<VisionMeasurement> getAcceptedMeasurements() {
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

  public boolean isConnected(String cameraName) {
    return getSample(cameraName).connected();
  }

  public boolean hasTarget(String cameraName) {
    return getSample(cameraName).hasTarget();
  }
}
