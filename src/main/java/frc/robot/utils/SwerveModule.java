// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Robot;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.constants.SwerveDriveConstants.S_MODULE_DETAILS;
import frc.robot.shufflecontrol.ShuffleTabController;
import frc.robot.utils.logger.Logger;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final SparkMax turnMotor;

  private final AbsoluteEncoder turnEncoder;

  private final VelocityVoltage driveController;
  private final SparkClosedLoopController turnController;

  // private final int moduleId;
  private final S_MODULE_DETAILS module_details;

  private int lastLimit = 0;

  private boolean isFlipped = false;

  private Rotation2d angularOffset = new Rotation2d(0);
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private Logger logger;
  private ShuffleTabController shuffleTab;

  /**
   * Constructs a new SwerveModule for a MAX Swerve Module housing a Falcon
   * driving motor and a Neo 550 Turning Motor.
   * 
   * @param drivingCANId  CAN ID for the driving motor
   * @param turningCANId  CAN ID for the turning motor
   * @param angularOffset Angular offset of the module in radians
   * @param shuffleTab    The shuffleboard tab to add widgets to
   */
  public SwerveModule(S_MODULE_DETAILS module_details, ShuffleTabController shuffleTab) {
    // moduleId = settings.CAN_ID_DRIVE;
    this.module_details = module_details;
    this.angularOffset = Rotation2d.fromRadians(module_details.ANGULAR_OFFSET);

    // --------------INIT--------------
    // create loggger
    logger = new Logger("swerve-" + module_details.CAN_ID_DRIVE,
        new String[] { "Pos", "Vel", "Ang", "Ang Rate", "Target Vel", "Target Ang" });

    // create shuffle tab
    this.shuffleTab = shuffleTab;
    shuffleTab
        .createWidget("Drive " + module_details.CAN_ID_DRIVE, BuiltInWidgets.kNumberBar,
            0 + (((module_details.CAN_ID_DRIVE - 1) % 2) * 3), 0 + (((module_details.CAN_ID_DRIVE - 1) / 2) * 2), 1, 2)
        .withProperties(Map.of("Min", -10, "Max", 10));
    shuffleTab
        .createWidget("Turn " + module_details.CAN_ID_STEER, BuiltInWidgets.kGyro,
            1 + (((module_details.CAN_ID_DRIVE - 1) % 2) * 3), 0 + (((module_details.CAN_ID_DRIVE - 1) / 2) * 2), 2, 2)
        .withProperties(Map.of("Starting Angle",
            this.angularOffset.unaryMinus().minus(Rotation2d.fromDegrees(270)).getDegrees() + 180));

    // --------------DRIVE MOTOR--------------
    driveMotor = new TalonFX(module_details.CAN_ID_DRIVE);
    final var driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.MotorOutput.Inverted = SwerveDriveConstants.DRIVE_MOTOR_INVERTED;
    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    /* set the Gear Ratio used for encoder reads */
    driveMotorConfig.Feedback.SensorToMechanismRatio = SwerveDriveConstants.DRIVE_GEAR_RATIO;
    /* enable and set Current Limiting to prevent brownouts */
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    /* set the PID Config for the drive motor */
    driveMotorConfig.Slot0.kP = SwerveDriveConstants.DRIVE_P;
    driveMotorConfig.Slot0.kI = SwerveDriveConstants.DRIVE_I;
    driveMotorConfig.Slot0.kD = SwerveDriveConstants.DRIVE_D;

    driveMotor.getConfigurator().apply(driveMotorConfig);
    driveController = new VelocityVoltage(0).withSlot(0);

    // --------------STEER MOTOR--------------
    turnMotor = new SparkMax(module_details.CAN_ID_STEER, MotorType.kBrushless);
    turnController = turnMotor.getClosedLoopController();
    final var turnMotorConfig = new SparkMaxConfig();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    turnEncoder = turnMotor.getAbsoluteEncoder();

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turnMotorConfig.absoluteEncoder.positionConversionFactor(SwerveDriveConstants.TURNING_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(SwerveDriveConstants.TURNING_ENCODER_VELOCITY_FACTOR);

    // Set the PID gains for the turning motor and
    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint
    turnMotorConfig.closedLoop
        .pidf(SwerveDriveConstants.TURNING_P, SwerveDriveConstants.TURNING_I, SwerveDriveConstants.TURNING_D,
            SwerveDriveConstants.TURNING_FF)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(SwerveDriveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
            SwerveDriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // enable and set Current Limiting to prevent brownouts
    turnMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    // Save the SPARK MAX configurations (Brownout protection).
    turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // --------------GO TO DEFAULTS--------------
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveMotor.setPosition(0);
  }

  /**
   * Returns the current raw state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        driveMotor.getVelocity().getValueAsDouble(),
        getRotation2d(false));
  }

  /**
   * Returns the current desired state of the module
   * 
   * @return The current desired state of the module
   */
  public SwerveModuleState getDesiredState(boolean optimizedAngle, boolean moduleRel) {
    SwerveModuleState state = optimizedAngle || !isFlipped ? desiredState
        : new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle);
    state = moduleRel ? state
        : new SwerveModuleState(state.speedMetersPerSecond,
            state.angle.plus(Rotation2d.fromRadians(module_details.ANGULAR_OFFSET)));
    return state;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    Rotation2d angle = getRotation2d(true).minus(angularOffset);
    // if the wheel is flipped, flip the returned angle too
    return new SwerveModulePosition(Math.abs(driveMotor.getPosition().getValueAsDouble()), angle);
  }

  public Rotation2d getRotation2d(boolean optimizedAngle) {
    // take care, get position only returns as rotations when a scale factor is not
    // set
    Rotation2d angle = Rotation2d.fromRadians(turnEncoder.getPosition());
    if (Robot.isSimulation()) {
      angle = desiredState.angle;
    }
    if (isFlipped)
      angle = angle.minus(Rotation2d.fromDegrees(180));
    return angle;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState(
        desiredState.speedMetersPerSecond,
        desiredState.angle.minus(angularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = /* SwerveModuleState. */optimize(
        correctedDesiredState,
        getRotation2d(false));

    driveMotor.setControl(
        driveController
            .withVelocity(
                // withVelocity accepts rps, not mps
                optimizedDesiredState.speedMetersPerSecond / SwerveDriveConstants.WHEEL_CIRCUMFERENCE_METERS));// .withFeedForward(DriveConstants.DRIVING_FF));
    turnController.setReference(
        optimizedDesiredState.angle.getRadians() + Math.PI,
        ControlType.kPosition);

    this.desiredState = optimizedDesiredState;

    logger.log(new double[] {
        driveMotor.getPosition().getValueAsDouble(),
        driveMotor.getVelocity().getValueAsDouble(),
        getRotation2d(false).getDegrees(),
        turnEncoder.getVelocity() * (180 / Math.PI),
        desiredState.speedMetersPerSecond,
        desiredState.angle.getDegrees()
    });
  }

  // private static final double intertia = 45;
  // TODO finish
  /* Optimises wheel angle, using an inertia based system *//*
                                                             * public SwerveModuleState optimize(SwerveModuleState
                                                             * desiredState, Rotation2d currentAngle) {
                                                             * var turnSpeed = 0;//Subsystems.swerveDrive.getTurnRate();
                                                             * double threshold = 0;//90 + MathUtil.clamp(turnSpeed *
                                                             * intertia, -80, 80);
                                                             * 
                                                             * var delta = desiredState.angle.minus(currentAngle);
                                                             * if (delta.getDegrees() > threshold || delta.getDegrees()
                                                             * < 180-threshold) {
                                                             * isFlipped = true;
                                                             * return new SwerveModuleState(
                                                             * -desiredState.speedMetersPerSecond,
                                                             * desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)
                                                             * ));
                                                             * } else {
                                                             * isFlipped = false;
                                                             * return new
                                                             * SwerveModuleState(desiredState.speedMetersPerSecond,
                                                             * desiredState.angle);
                                                             * }
                                                             * }
                                                             */

  /*
   * Optimises the wheel pivot direction to reduce time spent turning
   * uses a moving threshold to reduce flip-floping when near the 90deg point
   */
  public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    double error = Math.abs(delta.getDegrees());
    int limit = lastLimit;

    // optimizes by inverting the turn if the module is more than the limit
    if (error < limit) {
      lastLimit = error < 20 ? 90 : 135; // release only when near the target direction
      isFlipped = true;
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      isFlipped = false;
      lastLimit = error > 160 ? 90 : 45; // release only when near the inverted target direction
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  /* Updates the shuffleboard tab with new values */
  public void updateShuffleTab() {
    var state = desiredState;

    shuffleTab.getEntry("Turn " + module_details.CAN_ID_DRIVE).setDouble(state.angle.getDegrees());
    shuffleTab.getEntry("Drive " + module_details.CAN_ID_DRIVE).setDouble(state.speedMetersPerSecond);
  }

  /** Zeroes the drive encoder. */
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  /** reset turn motor pid I accumulation to 0 */
  public void resetIntegral() {
    turnController.setIAccum(0);
  }
}