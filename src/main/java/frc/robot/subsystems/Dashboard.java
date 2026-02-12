// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final NetworkTable drivingTable;
  private Auto auto;

  /** Creates a new Dashboard. */
  public Dashboard() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Scoring");
    drivingTable = inst.getTable("Driving PIDs");

    table.getEntry("leftL1").setBoolean(false);
    table.getEntry("leftL2").setBoolean(false);
    table.getEntry("leftL3").setBoolean(false);
    table.getEntry("leftL4").setBoolean(false);
    table.getEntry("rightL1").setBoolean(false);
    table.getEntry("rightL2").setBoolean(false);
    table.getEntry("rightL3").setBoolean(false);
    table.getEntry("rightL4").setBoolean(false);
    table.getEntry("blueLeft").setBoolean(false);
    table.getEntry("blueRight").setBoolean(false);
    table.getEntry("redLeft").setBoolean(false);
    table.getEntry("redRight").setBoolean(false);
    drivingTable.getEntry("useIncreasedFfValue").setBoolean(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    final Auto newAuto = getAuto();
    if (newAuto != auto) {
      setAuto(newAuto);
    }
  }

  public Auto getAuto() {
    if (table.getEntry("blueLeft").getBoolean(false) && auto != Auto.BLUE_LEFT) {
      return Auto.BLUE_LEFT;
    } else if (table.getEntry("blueRight").getBoolean(false) && auto != Auto.BLUE_RIGHT) {
      return Auto.BLUE_RIGHT;
    } else if (table.getEntry("redLeft").getBoolean(false) && auto != Auto.RED_LEFT) {
      return Auto.RED_LEFT;
    } else if (table.getEntry("redRight").getBoolean(false) && auto != Auto.RED_RIGHT) {
      return Auto.RED_RIGHT;
    }
    return auto;
  }

  public Auto getAutoFromDash() {
    if (table.getEntry("blueLeft").getBoolean(false)) {
      return Auto.BLUE_LEFT;
    } else if (table.getEntry("blueRight").getBoolean(false)) {
      return Auto.BLUE_RIGHT;
    } else if (table.getEntry("redLeft").getBoolean(false)) {
      return Auto.RED_LEFT;
    } else if (table.getEntry("redRight").getBoolean(false)) {
      return Auto.RED_RIGHT;
    }
    return auto;
  }

  public void clearAutoSelections() {
    table.getEntry("blueLeft").setBoolean(false);
    table.getEntry("blueRight").setBoolean(false);
    table.getEntry("redLeft").setBoolean(false);
    table.getEntry("redRight").setBoolean(false);
  }

  public void clearSelections() {
    table.getEntry("front").setBoolean(false);
    table.getEntry("front_left").setBoolean(false);
    table.getEntry("front_right").setBoolean(false);
    table.getEntry("back").setBoolean(false);
    table.getEntry("back_left").setBoolean(false);
    table.getEntry("back_right").setBoolean(false);
    table.getEntry("left").setBoolean(false);
    table.getEntry("right").setBoolean(false);
    table.getEntry("redLeft").setBoolean(false);
    table.getEntry("redRight").setBoolean(false);
    table.getEntry("blueLeft").setBoolean(false);
    table.getEntry("blueRight").setBoolean(false);
    table
        .getEntry("isRed")
        .setBoolean(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

    auto = null;
  }

  public void setAuto(Auto auto) {
    if (auto == null) {
      return;
    }

    clearAutoSelections();
    this.auto = auto;
    switch (auto) {
      case RED_LEFT:
        table.getEntry("redLeft").setBoolean(true);
        break;
      case RED_RIGHT:
        table.getEntry("redRight").setBoolean(true);
        break;
      case BLUE_LEFT:
        table.getEntry("blueLeft").setBoolean(true);
        break;
      case BLUE_RIGHT:
        table.getEntry("blueRight").setBoolean(true);
        break;
    }
  }

  public enum Auto {
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT
  }
}
