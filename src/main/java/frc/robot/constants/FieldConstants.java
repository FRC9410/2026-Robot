package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;

public class FieldConstants {
  public static final double X_MIN = 0.0;
  public static final double Y_MIN = 0.0;
  public static final double X_MAX = 17.5;
  public static final double Y_MAX = 8.0;
  public static final double TOL = 2.0;

  public static final double BLUE_START_X = 0.0;
  public static final double BLUE_END_X = 3.52;
  public static final double CENTER_START_X = 5.7;
  public static final double CENTER_END_X = 10.8;
  public static final double RED_START_X = 13.05;
  public static final double RED_END_X = 16.5;

  public static final double AUTO_LENGTH_IN_TIME = 15;
  public static final double TIME_BETWEEN_SHIFTS = 25;

  public static final Translation3d BLUE_TOP_CORNER = new Translation3d(0.5, 7.5, 0.0);
  public static final Translation3d BLUE_BOTTOM_CORNER = new Translation3d(0.5, 0.5, 0.0);
  
  public static final Translation3d RED_TOP_CORNER = new Translation3d(16, 7.5, 0.0);
  public static final Translation3d RED_BOTTOM_CORNER = new Translation3d(16, 0.5, 0.0);
}
