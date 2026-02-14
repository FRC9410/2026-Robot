// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;

public class Dashboard extends SubsystemBase {
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final NetworkTable drivingTable;
  private final NetworkTable testingTable;

  /** Creates a new Dashboard. */
  public Dashboard() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Scoring");
    drivingTable = inst.getTable("Driving PIDs");
    testingTable = inst.getTable("Robot Testing");



    testingTable.getEntry("robotState").setString(PowerRobotContainer.getData("robotState", "robotState is null").toString());
    testingTable.getEntry("robotPose").setValue(PowerRobotContainer.getData("robotPose", "robotPose is null").toString());
    testingTable.getEntry("ledColor").setString(PowerRobotContainer.getData("ledColor", "ledColor is null").toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run



    // Update Robot Testing dashboard: current state, pose, and auto routines
    //updateTestingDashboard();
  }

  private void updateTestingDashboard() {
    Object stateObj = PowerRobotContainer.getData("robotState");
    testingTable.getEntry("robotState").setString(stateObj != null ? stateObj.toString() : "UNKNOWN");

    Pose2d pose = PowerRobotContainer.getData("RobotPose", new Pose2d());
    testingTable.getEntry("poseX").setDouble(pose.getX());
    testingTable.getEntry("poseY").setDouble(pose.getY());
    testingTable.getEntry("poseRotationDegrees").setDouble(pose.getRotation().getDegrees());
  }

 
}