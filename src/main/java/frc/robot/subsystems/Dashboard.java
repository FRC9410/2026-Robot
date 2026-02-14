// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.RobotContainer;

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
    
    // velos
    testingTable.getEntry("spindexerVelocity").setInteger(PowerRobotContainer.getData("spindexerVelocity", 24));
    testingTable.getEntry("feederVelocity").setInteger(PowerRobotContainer.getData("feederVelocity", 24));
    testingTable.getEntry("shooterVelocity").setInteger(PowerRobotContainer.getData("shooterVelocity", 24));
  }

  @Override
  public void periodic() {
    updateTestingDashboard();
  }

  public double getValue (String key) {
    return testingTable.getEntry(key).getDouble(0.0);
  }

  private void updateTestingDashboard() {
    testingTable.getEntry("robotState").setString(PowerRobotContainer.getData("robotState", "robotState is null"));
    // testingTable.getEntry("robotPose").setValue(PowerRobotContainer.getData("robotPose", "robotPose is null"));
    testingTable.getEntry("ledColor").setString(PowerRobotContainer.getData("ledColor", "ledColor is null"));
    testingTable.getEntry("timeToShift").setDouble(PowerRobotContainer.getData("timeToShift", -1.0));

    testingTable.getEntry("matchTime").setDouble(DriverStation.getMatchTime());
    testingTable.getEntry("matchLocation").setValue(DriverStation.getLocation().getAsInt());
    testingTable.getEntry("matchMessage").setString(DriverStation.getGameSpecificMessage());
  }
}