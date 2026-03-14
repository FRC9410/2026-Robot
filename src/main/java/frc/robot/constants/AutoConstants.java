package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {
  public static final double TRANSLATION_TOLERANCE = 0.05;
  public static final double ROTATION_TOLERANCE = 1.0;

  public static final Pose2d TEST_POSITION = new Pose2d(5, 0, Rotation2d.fromDegrees(90.0));
  public static final Pose2d TEST_POSITION2 = new Pose2d(0, 0.0254, Rotation2d.fromDegrees(0.0));
  public static final Pose2d TEST_POSITION3 = new Pose2d(0, -0.0254, Rotation2d.fromDegrees(0.0));

  public static final Pose2d RED_HP_1 = new Pose2d(13.26, 5.7, Rotation2d.fromDegrees(45.0));
  public static final Pose2d RED_HP_2 = new Pose2d(10.5, 5.7, Rotation2d.fromDegrees(45.0));
  public static final Pose2d RED_HP_3 = new Pose2d(9.55, 7.27, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_HP_4 = new Pose2d(8.2,7, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_HP_5 = new Pose2d(8.2,5.5, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_HP_6 = new Pose2d(10.5,5.7, Rotation2d.fromDegrees(45.0));
  public static final Pose2d RED_HP_7 = new Pose2d(13.26,5.7, Rotation2d.fromDegrees(-135.0));
  // public static final Pose2d RED_HP_8 = new Pose2d(15.89,7.07, Rotation2d.fromDegrees(-90.0));
  
  public static final Pose2d RED_DEPOT_1 = new Pose2d(13.26,2.5, Rotation2d.fromDegrees(0.0));
  public static final Pose2d RED_DEPOT_2 = new Pose2d(10.5,2.5, Rotation2d.fromDegrees(0.0));
  public static final Pose2d RED_DEPOT_3 = new Pose2d(9.55,0.73, Rotation2d.fromDegrees(0.0));
  public static final Pose2d RED_DEPOT_4 = new Pose2d(8.61,1, Rotation2d.fromDegrees(0.0));
  public static final Pose2d RED_DEPOT_5 = new Pose2d(8.61,3, Rotation2d.fromDegrees(0.0));
  public static final Pose2d RED_DEPOT_6 = new Pose2d(10.5,2.5, Rotation2d.fromDegrees(0.0));



  

}
