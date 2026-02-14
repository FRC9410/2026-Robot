package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

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
}
