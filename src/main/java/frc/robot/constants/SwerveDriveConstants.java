
package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.RangeMath.AxesFit;
import frc.robot.utils.RangeMath.DriveBaseFit;

// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java
// You can reference the above URL if you want to check the validity of any non-robot-specific constants.

// NOTE - As of 07/02/2024, all constants in this file are either from the template (URL above) or just placeholder
// values. Therefore, most of these are completely innaccurate! Please update them to be accurate before testing.
// - Neel

public class SwerveDriveConstants {
  // NEO Motor Constants
  /** Free speed of the driving motor in rpm */
  public static final double FREE_SPEED_RPM = 6380;

  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  /** Max speed of robot in meters per second */
  public static final double MAX_SPEED = 4.8; // TODO check this
  /** Max acceleration of robot in meters per second squared */
  public static final double MAX_ACCELERATION = 1; // TODO check this
  /** Max angular speed of robot in radians per second */
  public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
  /** Max angular acceleration of robot in radians per second squared */
  public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_SPEED / 60;

  /** Direction slew rate in radians per second */
  public static final double DIRECTION_SLEW_RATE = 1.2;
  /** Magnitude slew rate in percent per second (1 = 100%) */
  public static final double MAGNITUDE_SLEW_RATE = 1.8;
  /** Rotational slew rate in percent per second (1 = 100%) */
  public static final double ROTATIONAL_SLEW_RATE = 8.0;

  /** Gear ratio of the MAX Swerve Module driving motor */
  public static final double DRIVE_GEAR_RATIO = 4.71;

  // Chassis configuration
  /** Distance between front and back wheel on robot in meters */
  public static final double TRACK_WIDTH = Units.inchesToMeters(20.7);
  /** Distance between centers of left and right wheels on robot in meters */
  public static final double WHEEL_BASE = Units.inchesToMeters(20.7);

  /**
   * Position of front left swerve module relative to robot center. Mainly for sim
   * purposes.
   */
  // private static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new
  // Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2);
  /**
   * Position of front right swerve module relative to robot center. Mainly for
   * sim purposes.
   */
  // private static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new
  // Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2);
  /**
   * Position of back left swerve module relative to robot center. Mainly for sim
   * purposes.
   */
  // private static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new
  // Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2);
  /**
   * Position of back right swerve module relative to robot center. Mainly for sim
   * purposes.
   */
  // private static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new
  // Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2);

  // Angular offsets of the modules relative to the chassis in radians
  /** Angular offset of front left module relative to chassis */
  // public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = 0;
  /** Angular offset of front right module relative to chassis */
  // public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;
  /** Angular offset of back left module relative to chassis */
  // public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
  /** Angular offset of back right module relative to chassis */
  // public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI;

  // SPARK MAX CAN IDs
  /** CAN ID for Front Left Module Driving Motor */
  // public static final int FRONT_LEFT_DRIVING_CAN_ID = 1;
  /** CAN ID for Front Right Module Driving Motor */
  // public static final int FRONT_RIGHT_DRIVING_CAN_ID = 2;
  /** CAN ID for Back Left Module Driving Motor */
  // public static final int BACK_LEFT_DRIVING_CAN_ID = 3;
  /** CAN ID for Back Right Module Driving Motor */
  // public static final int BACK_RIGHT_DRIVING_CAN_ID = 4;

  /** CAN ID for Front Left Module Turning Motor */
  // public static final int FRONT_LEFT_TURNING_CAN_ID = 1;
  /** CAN ID for Front Right Module Turning Motor */
  // public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
  /** CAN ID for Back Left Module Turning Motor */
  // public static final int BACK_LEFT_TURNING_CAN_ID = 3;
  /** CAN ID for Back Right Module Turning Motor */
  // public static final int BACK_RIGHT_TURNING_CAN_ID = 4;

  /** IMU Gyro Inversion */
  public static final boolean GYRO_REVERSED = false;

  // Module Constants
  /** Drive motor inversion. */
  public static final InvertedValue DRIVE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

  /**
   * Whether the turning encoder is inverted or not.
   * 
   * In the MAXSwerve module, this should be set to `true`, since the output shaft
   * rotates in the opposite direction of the steering motor.
   */
  public static final boolean TURNING_ENCODER_INVERTED = true;

  /**
   * Gear attached direct to the drive motor
   * The MAXSwerve module can be configured with several different gears.
   * This changes the drive speed of the module
   * [L:12,M:13,H:14,EH1:14,EH2:14,EH3:15,EH4:16,EH5:16]
   */
  public static final int DRIVING_MOTOR_PINION_TEETH = 15;
  /**
   * Gear driven by the motor spur
   * The MAXSwerve module can be configured with several different gears.
   * This changes the drive speed of the module
   * [L:22,M:22,H:22,EH1:21,EH2:20,EH3:20,EH4:20,EH5:19]
   */
  public static final int DRIVING_MOTOR_SPUR_TEETH = 20;

  // Calculations required for driving motor conversion factors and feed forward
  /** Free speed of the driving motor in rps */
  public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
  /** Wheel diameter in meters */
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
  /** Wheel circumference in meters */
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
  /** Driving distance per motor revolution, in meters */
  public static final double DRIVE_DISTANCE_PER_MOTOR_REVOLUTION = WHEEL_CIRCUMFERENCE_METERS
      / (2048 * DRIVE_GEAR_RATIO); // TODO: Why is 2048 there? I think there's a reason, but idk

  /**
   * 45 teeth on the wheel's bevel gear, 15 teeth on the bevel pinion
   */
  public static final double DRIVING_MOTOR_REDUCTION = ((double) DRIVING_MOTOR_SPUR_TEETH / DRIVING_MOTOR_PINION_TEETH)
      * (45.0 / 15.0);

  /** Free speed of the wheel in rps */
  public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
      * WHEEL_CIRCUMFERENCE_METERS)
      / DRIVING_MOTOR_REDUCTION;

  /** Driving encoder position factor */
  public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
      / DRIVING_MOTOR_REDUCTION; // meters
  /** Driving encoder velocity factor */
  public static final double DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR / 60.0; // m/s

  /** Turning encoder position factor */
  public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
  /** Turning encoder velocity factor */
  public static final double TURNING_ENCODER_VELOCITY_FACTOR = TURNING_ENCODER_POSITION_FACTOR / 60.0; // rad/s
  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

  // TODO tune PID
  public static final double DRIVE_P = 0.7;
  public static final double DRIVE_I = 0.0;// 5;
  public static final double DRIVE_D = 0.05;
  public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;

  public static final double TURNING_P = 0.25;
  public static final double TURNING_I = 0.001;
  public static final double TURNING_D = 0.02;
  public static final double TURNING_FF = 0;// .1;

  // Auto Constants
  /** Auto translation PID constants */
  public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(0.1, 0, 0);
  /** Auto rotation PID constants */
  public static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(0.25, 0, 0);
  /** Auto module max speed in m/s */
  public static final double MAX_MODULE_SPEED = 4.5;
  /** Drivebase radius in m (distance from center of robot to farthest module) */
  public static final double DRIVEBASE_RADIUS = Math.sqrt(Math.pow(WHEEL_BASE / 2, 2) + Math.pow(TRACK_WIDTH / 2, 2));

  public static final double updateShuffleInterval = 0.1 / 0.02; /* (x / 20ms) */

  /**
   * Auto angular speed and acceleration constraints in radians per second and
   * radians per second squared
   */
  public static final TrapezoidProfile.Constraints AUTO_THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED,
      MAX_ANGULAR_ACCELERATION);

  /*
   * public static DriveBaseFit PILOT_SETTINGS = DriveBaseFit(
   * 0, 1, 4, 0.1, true,
   * 0, 1, 4, 0.1, false,
   * 0, 1, 3, 0.1, false,
   * 0.85, 0.8);
   */
  public static DriveBaseFit PILOT_SETTINGS = new DriveBaseFit(
      new AxesFit().withOutputMinMax(0, 0.7).withPow(4).withDeadBand(0.1)
          .withLimiter(0.15).withBooster(1),
      new AxesFit().withPow(3).withDeadBand(0.1).withLimiter(0.15));

  /*
   * public static DriveBaseFit PILOT_DEMO_SETTINGS = DriveBaseFit.InitSwerveBot(
   * 0, 0.2, 2, 0.1, true,
   * 0, 0.2, 2, 0.1, false,
   * 0, 0.2, 2, 0.1, false,
   * 0.6, 1);
   */
  public static DriveBaseFit PILOT_DEMO_SETTINGS = new DriveBaseFit(
      new AxesFit().withOutputMinMax(0, 0.2).withPow(2).withDeadBand(0.1).withLimiter(0.15),
      new AxesFit().withOutputMinMax(0, 0.2).withPow(2).withDeadBand(0.1).withLimiter(0.15));

  public static final S_MODULE_DETAILS SWERVE_MODULE_FL = new S_MODULE_DETAILS(
      1, // Drive motor CAN ID
      1, // Steer motor CAN ID
      0, // Offset rel to FL module
      new Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2) // location rel to centre
  );
  public static final S_MODULE_DETAILS SWERVE_MODULE_FR = new S_MODULE_DETAILS(
      2, // Drive motor CAN ID
      2, // Steer motor CAN ID
      Math.PI / 2, // Offset rel to FL module
      new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2) // location rel to centre
  );
  public static final S_MODULE_DETAILS SWERVE_MODULE_BL = new S_MODULE_DETAILS(
      3, // Drive motor CAN ID
      3, // Steer motor CAN ID
      -Math.PI / 2, // Offset rel to FL module
      new Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2) // location rel to centre
  );
  public static final S_MODULE_DETAILS SWERVE_MODULE_BR = new S_MODULE_DETAILS(
      4, // Drive motor CAN ID
      4, // Steer motor CAN ID
      Math.PI, // Offset rel to FL module
      new Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2) // location rel to centre
  );

  /** Swerve Kinematics */
  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      SWERVE_MODULE_FL.LOCATION,
      SWERVE_MODULE_FR.LOCATION,
      SWERVE_MODULE_BL.LOCATION,
      SWERVE_MODULE_BR.LOCATION);

  public static class S_MODULE_DETAILS {
    /** CAN ID for the module's Driving Motor */
    public final int CAN_ID_DRIVE;
    /** CAN ID for the module's Steering Motor */
    public final int CAN_ID_STEER;
    /**
     * Angular offset of the module around the robot's center, FL module is at 0,
     * top down CW positive.
     */
    public final double ANGULAR_OFFSET;
    /**
     * Location of module relative to robot center, top down, right and front pos.
     * Mainly for sim purposes.
     */
    public final Translation2d LOCATION;

    protected S_MODULE_DETAILS(int CAN_ID_DRIVE, int CAN_ID_STEER, double ANGULAR_OFFSET, Translation2d LOCATION) {
      this.CAN_ID_DRIVE = CAN_ID_DRIVE;
      this.CAN_ID_STEER = CAN_ID_STEER;
      this.ANGULAR_OFFSET = ANGULAR_OFFSET;
      this.LOCATION = LOCATION;
    }
  }
}
