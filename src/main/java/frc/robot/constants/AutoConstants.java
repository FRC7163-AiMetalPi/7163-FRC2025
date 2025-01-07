package frc.robot.constants;

import frc.robot.utils.PIDControllerSupplier;

public class AutoConstants {
  // TODO: tune these
  /** Auto translation PID constants */
  public static final PIDControllerSupplier TRANSLATION_PID = new PIDControllerSupplier(0.5, 0, 0);
  /** Auto rotation PID constants */
  public static final PIDControllerSupplier ROT_PID = new PIDControllerSupplier(0.5, 0, 0);
  /** Tolerance for translation PID in meters */
  public static final double TRANSLATION_TOLERANCE = 0.1;
  /** Tolerance for rotation PID in radians */
  public static final double ROT_TOLERANCE = 0.1;
}
