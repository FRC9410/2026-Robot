// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

/** Subsystem for vision (Limelight) – targets and robot pose. */
public class Vision extends SubsystemBase {

  private final NetworkTable leftLimelight;
  private final NetworkTable rightLimelight;
  private final NetworkTable turretLimelight;
  final List<Integer> blueTagIds = Arrays.asList(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  final List<Integer> redTagIds = Arrays.asList(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  final List<Integer> tagIds;

  public Vision() {
    leftLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.LEFT_TABLE);
    rightLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.RIGHT_TABLE);
    turretLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.TURRET_TABLE);

    tagIds =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redTagIds : blueTagIds;
  }

  /**
   * Get the primary Limelight table (e.g. "limelight"). For dual cameras use
   * getTable(LEFT_TABLE) / getTable(RIGHT_TABLE) in logic.
   */
  public static NetworkTable getTable(String name) {
    return NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void periodic() {  
    PowerRobotContainer.setData("robotPose", getRobotPose(getBestLimelight()));
  }

  /** Whether the camera has at least one valid target. */
  public boolean hasTarget() {
    return getTV() > 0.5;
  }

  /** Horizontal offset from crosshair to target (degrees). */
  public double getTX() {
    return getBestLimelight().getEntry("tx").getDouble(0);
  }

  /** Vertical offset from crosshair to target (degrees). */
  public double getTY() {
    return getBestLimelight().getEntry("ty").getDouble(0);
  }

  /** Target area (0–100%). */
  public double getTA() {
    return getBestLimelight().getEntry("ta").getDouble(0);
  }

  /** Valid target (1 = valid). */
  public double getTV() {
    return getBestLimelight().getEntry("tv").getDouble(0);
  }

  // TODO: fix this because the robot moves lol
  /** Robot pose from Limelight (when using 2D or 3D pose). */
  public Pose2d getRobotPose(NetworkTable table) {
    // double[] arr = table.getEntry("botpose").getDoubleArray(new double[6]);
    // if (arr.length >= 6) {
    //   return new Pose2d(
    //       new Translation2d(arr[0], arr[1]),
    //       Rotation2d.fromDegrees(arr[5]));
    // }
    return new Pose2d(10.0, 10.0, new Rotation2d(10.0));
  }

  /** Set pipeline index (0-based). */
  public void setPipeline(int index) {
    getBestLimelight().getEntry("pipeline").setNumber(index);
  }

  /** Set LED mode: 0=current pipeline, 1=force off, 2=force blink, 3=force on. */
  public void setLEDMode(int mode) {
    getBestLimelight().getEntry("ledMode").setNumber(mode);
  }

  public int getTagId(NetworkTable table) {
    return (int) table.getEntry("tid").getInteger(0);
  }

  public NetworkTable getBestLimelight() {
    final NetworkTable leftLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.LEFT_TABLE);
    final NetworkTable rightLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.RIGHT_TABLE);
    final NetworkTable turretLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.TURRET_TABLE);

    LimelightHelpers.PoseEstimate leftPerimeterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    LimelightHelpers.PoseEstimate rightPerimeterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
    LimelightHelpers.PoseEstimate turretPerimeterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret");

    NetworkTable bestLimelight = leftLimelight;
    double bestArea = 0;

    if (turretPerimeterMeasurement != null && tagIds.contains(getTagId(turretLimelight))
        && turretPerimeterMeasurement.avgTagArea > bestArea) {
      bestLimelight = turretLimelight;
      bestArea = turretPerimeterMeasurement.avgTagArea;
    }

    if (leftPerimeterMeasurement != null && tagIds.contains(getTagId(leftLimelight))
        && leftPerimeterMeasurement.avgTagArea > bestArea) {
      bestLimelight = leftLimelight;
      bestArea = leftPerimeterMeasurement.avgTagArea;
    }

    if (rightPerimeterMeasurement != null && tagIds.contains(getTagId(rightLimelight))
        && rightPerimeterMeasurement.avgTagArea > bestArea) {
      bestLimelight = rightLimelight;
      bestArea = rightPerimeterMeasurement.avgTagArea;
    }

    return bestLimelight;
  }

  
    // Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(bestLimelight);
    // LimelightHelpers.PoseEstimate bestMeasurement =
    //     LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimelight);

    // if (bestMeasurement != null && bestMeasurement.avgTagArea > 0.1) {
    //   Pose2d newPose = pose.toPose2d();
    //   drive.resetRotation(newPose.getRotation());
    //   LimelightHelpers.SetRobotOrientation(
    //       "limelight-left", newPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //   LimelightHelpers.SetRobotOrientation(
    //       "limelight-right", newPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    // } else {
    //   return;
    // }
}
