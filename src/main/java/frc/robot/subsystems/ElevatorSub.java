package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSub extends SubsystemBase {
  private final TalonFX motor = new TalonFX(ElevatorConstants.MOTOR_ID);
  private final Encoder encoder = ElevatorConstants.ENCODER_ID.get();
  private final ProfiledPIDController controller = new ProfiledPIDController(
      ElevatorConstants.CONTROLLER_P,
      ElevatorConstants.CONTROLLER_I,
      ElevatorConstants.CONTROLLER_D,
      ElevatorConstants.MOTION_CONSTRAINTS);

  private boolean eStopped = false;

  public ElevatorSub() {
    super();

    final var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motor.getConfigurator().apply(motorConfig);

    controller.setTolerance(ElevatorConstants.POSITION_TOLERANCE);

    SmartDashboard.putData("Elevator Motor", motor);
    SmartDashboard.putData("Elevator Encoder", encoder);
    SmartDashboard.putData("Elevator Controller", controller);
    SmartDashboard.putBoolean("Elevator E-Stopped", eStopped);
  }

  public double getPosition() {
    return encoder.getDistance();
  }

  public void setTargetPosition(double position) {
    controller.setGoal(position);
  }

  public double getTargetPosition() {
    return controller.getGoal().position;
  }

  public boolean atTargetPosition() {
    return controller.atGoal();
  }

  private boolean shouldEStop() {
    final var encoderPosition = getPosition();
    final var motorPosition = motor.getPosition().getValueAsDouble();

    return encoderPosition < 0
        || encoderPosition > ElevatorConstants.MAX_ALLOWABLE_POSITION
        || motorPosition < 0
        || motorPosition > ElevatorConstants.MAX_ALLOWABLE_POSITION;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elevator E-Stopped", eStopped);

    if (eStopped || shouldEStop()) {
      motor.set(0);
      return;
    }

    final var position = getPosition();
    var out = controller.calculate(position);
    out = MathUtil.clamp(out, -1, 1);
    motor.set(out);
  }

}
