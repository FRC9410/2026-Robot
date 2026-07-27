// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

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
