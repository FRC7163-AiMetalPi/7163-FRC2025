package frc.robot.constants;

public class LoggerConstants {
  /** limit for repeated attempts to create each log file on USB storage */
  public static final int REPEAT_LIMIT_LOGGER_CREATION = 128;
  /** limit for repeated attempts to create parent folder on USB storage */  
  public static final int REPEAT_LIMIT_LOGGER_FOLDER_CREATION = 128;
  /** limit for repeated attempts to read auto from internal storage */
  public static final int REPEAT_LIMIT_AUTO_READ = 10;
  /** save attempts per second for the logger */
  public static final int SAVE_RATE = 10;
}
