// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/**
 * Field extents used as a sanity gate on vision poses. Tolerance allows a small amount of overhang
 * past the field edge (a real pose can sit slightly outside when the robot is against a wall)
 * without accepting obvious garbage.
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
