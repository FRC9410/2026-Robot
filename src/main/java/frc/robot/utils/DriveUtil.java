package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.TunerConstants;

public class DriveUtil {

  public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double MAX_DRIVE_TO_POINT_SPEED =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75;
  public static final double SLOW_DRIVE_TO_POINT_SPEED =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75 / 4;
  public static final double STATIC_FRICTION_CONSTANT = 0.085;

  public static boolean isClose(Pose2d currentPose, Pose2d targetPose) {
    final Translation2d translationToPoint =
        currentPose.getTranslation().minus(targetPose.getTranslation());
    final double linearDistance = translationToPoint.getNorm();
    return linearDistance < 1; // meters
  }

  /**
   * Calculates the x and y velocity components to drive from currentPose toward targetPose.
   *
   * @param currentPose the robot's current pose
   * @param targetPose the desired target pose
   * @param directionMultiplier alliance-based sign flip (-1 for blue, 1 for red)
   * @param driveToPointController PID controller for distance
   * @param poseTolerance the current pose tolerance value
   * @return a Translation2d whose x/y are the field-relative velocity components
   */
  public static Translation2d calculateDriveToPointVelocity(
      Pose2d currentPose,
      Pose2d targetPose,
      double directionMultiplier,
      PIDController driveToPointController,
      double poseTolerance) {

    final Translation2d translationToPoint =
        currentPose.getTranslation().minus(targetPose.getTranslation());
    final double linearDistance = translationToPoint.getNorm();

    double ff = 0;
    if (linearDistance >= Units.inchesToMeters(0.5)) {
      ff = STATIC_FRICTION_CONSTANT * MAX_SPEED;
    }

    double cappedSpeed = isClose(currentPose, targetPose) && poseTolerance < 6
        ? SLOW_DRIVE_TO_POINT_SPEED : MAX_DRIVE_TO_POINT_SPEED;

    final Rotation2d directionOfTravel = translationToPoint.getAngle();
    final double velocity =
        Math.min(Math.abs(driveToPointController.calculate(linearDistance, 0)) + ff, cappedSpeed);

    final double xSpeed = velocity * directionOfTravel.getCos() * directionMultiplier;
    final double ySpeed = velocity * directionOfTravel.getSin() * directionMultiplier;

    return new Translation2d(xSpeed, ySpeed);
  }
}
