package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {
  public static final double TRANSLATION_TOLERANCE = 0.05;
  public static final double ROTATION_TOLERANCE = 1.0;

  public static final Pose2d TEST_POSITION = new Pose2d(5, 0, Rotation2d.fromDegrees(90.0));
  public static final Pose2d TEST_POSITION2 = new Pose2d(0, 0.0254, Rotation2d.fromDegrees(0.0));
  public static final Pose2d TEST_POSITION3 = new Pose2d(0, -0.0254, Rotation2d.fromDegrees(0.0));
}
