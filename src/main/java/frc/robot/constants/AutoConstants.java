package frc.robot.constants;

public class AutoConstants {
  // TODO: tune these

  // Auto translation PID constants
  public static final double TRANSLATION_P = 0.5;
  public static final double TRANSLATION_I = 0;
  public static final double TRANSLATION_D = 0;

  // Auto rotation PID constants
  public static final double ROT_P = 0.5;
  public static final double ROT_I = 0;
  public static final double ROT_D = 0;

  /** Tolerance for translation PID in meters */
  public static final double TRANSLATION_TOLERANCE = 0.1;
  /** Tolerance for rotation PID in radians */
  public static final double ROT_TOLERANCE = 0.1;
}
