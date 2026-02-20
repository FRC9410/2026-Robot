package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.constants.TurretConstants;

public class TurretHelpers {
    /**
     * Gets the position of the limelight on the turret relative to center of robot.
     * 
     * @param turret_dir In degrees
     * @return
     */
    public static Pose2d turretCamPosRelative (double turretDegrees) {
        double turretRadians = Math.toRadians(turretDegrees);

        return new Pose2d(
            Constants.Turret.TURRET_DIST_FROM_ROBOT_CENTER + Math.cos(turretRadians) * Constants.Turret.TURRET_RADIUS,
            Math.sin(turretRadians) * Constants.Turret.TURRET_RADIUS,
            Rotation2d.fromRadians(turretRadians)
        );
    }

    /// DOES NOT PREDICT FOR VELO
    public static double getRotationToTargetDirect () {
        return 0.0;
    }

    public static double calculateHoodAngle(double distance) {
        double hoodAngleSetpoint = TurretConstants.hoodAngleInterpolator.getInterpolatedValue(distance);
        return hoodAngleSetpoint;
    }

    public static double calculateShooterVelocity(double distance) {
        double shooterVelocitySetpoint = TurretConstants.shooterVelocityInterpolator.getInterpolatedValue(distance);
        return shooterVelocitySetpoint;
    }

    /// DOES NOT PREDICT FOR VELO
    public static double getElevationToTarget () {
        return 0.0;
    }

    /// sets prc.data("turretTarget") as Translation3d
    public static void setTarget (Translation3d pos) {
        PowerRobotContainer.setData("turretTarget", pos);
    }
    

    /// does nothing as of now
    public static double predictRotationToTarget () {
        return 0.0;
    }

    public static void aimTurret() {
        
    }
}