// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


// A dropdown selector backed entirely by raw NetworkTables entries. Publishes the
// standard "String Chooser" schema, which Elastic, AdvantageScope and the sim GUI
// all render as a combo box.
public class NTChooser<T> {
  private final Map<String, T> options = new LinkedHashMap<>();
  private final NetworkTableEntry optionsEntry;
  private final NetworkTableEntry defaultEntry;
  private final NetworkTableEntry activeEntry;
  private final NetworkTableEntry selectedEntry;

  private String defaultName = "";

  public NTChooser(String tablePath) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tablePath);
    table.getEntry(".type").setString("String Chooser");
    table.getEntry(".controllable").setBoolean(true);
    optionsEntry = table.getEntry("options");
    defaultEntry = table.getEntry("default");
    activeEntry = table.getEntry("active");
    selectedEntry = table.getEntry("selected");
  }

  public void addOption(String name, T value) {
    options.put(name, value);
    publishOptions();
  }

  public void setDefaultOption(String name, T value) {
    defaultName = name;
    options.put(name, value);
    defaultEntry.setString(name);
    // setDefault* only writes if nothing has claimed the topic, so a dashboard
    // that reconnects and re-sends its selection is never stomped.
    selectedEntry.setDefaultString(name);
    publishOptions();
  }

  // Selection from the dashboard, falling back to the default if it is unset or unknown. 
  public String getSelectedName() {
    String selected = selectedEntry.getString(defaultName);
    return options.containsKey(selected) ? selected : defaultName;
  }

  public T get() {
    return options.get(getSelectedName());
  }

  /** Call periodically so the dashboard echoes what the robot actually resolved. */
  public void publishActive() {
    activeEntry.setString(getSelectedName());
  }

  private void publishOptions() {
    optionsEntry.setStringArray(options.keySet().toArray(new String[0]));
  }
}
