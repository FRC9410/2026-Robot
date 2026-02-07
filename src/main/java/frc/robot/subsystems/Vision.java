// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/** Subsystem for vision (Limelight) – targets and robot pose. */
public class Vision extends SubsystemBase {

  private final NetworkTable table;
  final List<Integer> blueTagIds = Arrays.asList(12, 13, 16, 17, 18, 19, 20, 21, 22); //needs new tagIDs probably
  final List<Integer> redTagIds = Arrays.asList(1, 2, 3, 6, 7, 8, 9, 10, 11);
  final List<Integer> tagIds;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.CAMERA_NAME);

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
  public void periodic() {}

  /** Whether the camera has at least one valid target. */
  public boolean hasTarget() {
    return getTV() > 0.5;
  }

  /** Horizontal offset from crosshair to target (degrees). */
  public double getTX() {
    return table.getEntry("tx").getDouble(0);
  }

  /** Vertical offset from crosshair to target (degrees). */
  public double getTY() {
    return table.getEntry("ty").getDouble(0);
  }

  /** Target area (0–100%). */
  public double getTA() {
    return table.getEntry("ta").getDouble(0);
  }

  /** Valid target (1 = valid). */
  public double getTV() {
    return table.getEntry("tv").getDouble(0);
  }

  /** Robot pose from Limelight (when using 2D or 3D pose). */
  public Pose2d getRobotPose() {
    double[] arr = table.getEntry("botpose").getDoubleArray(new double[6]);
    if (arr.length >= 6) {
      return new Pose2d(
          new Translation2d(arr[0], arr[1]),
          Rotation2d.fromDegrees(arr[5]));
    }
    return new Pose2d();
  }


  /** Set pipeline index (0-based). */
  public void setPipeline(int index) {
    table.getEntry("pipeline").setNumber(index);
  }

  /** Set LED mode: 0=current pipeline, 1=force off, 2=force blink, 3=force on. */
  public void setLEDMode(int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }

  public int getTagId(NetworkTable table) {
    return (int) table.getEntry("tid").getInteger(0);
  }


  public String getBestLimelight() {
    final NetworkTable leftLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_TABLE);
    final NetworkTable rightLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_TABLE);
    final NetworkTable turretLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.TURRET_TABLE);

    LimelightHelpers.PoseEstimate leftPerimeterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    LimelightHelpers.PoseEstimate rightPerimeterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
    LimelightHelpers.PoseEstimate turretPerimeterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret");

    String bestLimelight = "";
    double bestArea = 0;

    if (turretPerimeterMeasurement != null && tagIds.contains(getTagId(turretLimelight))
        && turretPerimeterMeasurement.avgTagArea > bestArea) {
      bestLimelight = "limelight-turret";
      bestArea = turretPerimeterMeasurement.avgTagArea;
    }

    if (leftPerimeterMeasurement != null && tagIds.contains(getTagId(leftLimelight))
        && leftPerimeterMeasurement.avgTagArea > bestArea) {
      bestLimelight = "limelight-left";
      bestArea = leftPerimeterMeasurement.avgTagArea;
    }

    if (rightPerimeterMeasurement != null && tagIds.contains(getTagId(rightLimelight))
        && rightPerimeterMeasurement.avgTagArea > bestArea) {
      bestLimelight = "limelight-right";
      bestArea = rightPerimeterMeasurement.avgTagArea;
    }

    return bestLimelight;
  }

}
