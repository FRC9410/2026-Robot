package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.StateMachine;

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

    public static void aimTurret(StateMachine stateMachine, double position) {
        var turret = stateMachine.turret;
        

    }

    public static double getDistance(Pose2d RobotPosition, Translation2d hopperPosition) {
        Translation2d difference = hopperPosition.minus(RobotPosition.getTranslation());
        double distance = Math.sqrt(difference.getX() * difference.getX() + difference.getY() * difference.getY()); //it is written like this because it makes Caden mad :)
        return distance;
    }

    // TODO: this
    public static double getTangentalSpeed(ChassisSpeeds speeds, Pose2d robotPos) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double relativeX = Constants.Field.HOPPER.getX() - robotPos.getX();
        double relativeY = Constants.Field.HOPPER.getY() - robotPos.getY();

        double relativeMagPos = Math.sqrt(relativeX*relativeX + relativeY*relativeY);
        double magVelocity = Math.sqrt(vx * vx + vy * vy);
        double normalizedX = relativeX / relativeMagPos;
        double normalizedY = relativeY / relativeMagPos;

        return Math.sqrt(magVelocity * magVelocity - Math.pow(vx * normalizedX + vy * normalizedY, 2));
    }

    public static double getRadialSpeed(ChassisSpeeds speeds, Pose2d robotPos) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double relativeX = Constants.Field.HOPPER.getX() - robotPos.getX();
        double relativeY = Constants.Field.HOPPER.getY() - robotPos.getY();

        double relativeMagPos = Math.sqrt(relativeX*relativeX + relativeY*relativeY);
        double normalizedX = relativeX / relativeMagPos;
        double normalizedY = relativeY / relativeMagPos;

        return vx * normalizedX + vy * normalizedY;
    }

    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public static Translation2d project(Translation2d a, Translation2d onto) {
        return onto.times(dot(a, onto) / dot(onto, onto));
    }

    public static Translation2d reject(Translation2d a, Translation2d onto) {
        return a.minus(project(a, onto));
    }

    public static Translation2d elementMult(Translation2d a, Translation2d b) {
        return new Translation2d(a.getX() * b.getX(), a.getY() * b.getY());
    }
}
