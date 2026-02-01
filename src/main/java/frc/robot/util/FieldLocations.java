// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

/** Add your docs here. */
public class FieldLocations {
  public static boolean isNearHp(Pose2d currentPose) {
    Translation2d[] corners = {
      new Translation2d(FieldConstants.X_MIN, FieldConstants.Y_MIN),
      new Translation2d(FieldConstants.X_MIN, FieldConstants.Y_MAX),
      new Translation2d(FieldConstants.X_MAX, FieldConstants.Y_MIN),
      new Translation2d(FieldConstants.X_MAX, FieldConstants.Y_MAX)
    };
    Translation2d position = currentPose.getTranslation();
    for (Translation2d corner : corners) {
      if (Math.abs(position.getX() - corner.getX()) < FieldConstants.TOL
          && Math.abs(position.getY() - corner.getY()) < FieldConstants.TOL) {
        return true;
      }
    }
    return false;
  }
}
