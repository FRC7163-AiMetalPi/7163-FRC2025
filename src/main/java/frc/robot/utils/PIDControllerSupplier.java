package frc.robot.utils;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

/** Utility class to build a {@link PIDController} */
public class PIDControllerSupplier implements Supplier<PIDController> {
  private final double kp;
  private final double ki;
  private final double kd;
  private Optional<Double> setpoint;
  private Optional<Double> tolerance;

  public PIDControllerSupplier(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.setpoint = Optional.empty();
    this.tolerance = Optional.empty();
  }

  public PIDControllerSupplier withSetpoint(double setpoint) {
    this.setpoint = Optional.of(setpoint);
    return this;
  }

  public PIDControllerSupplier withTolerance(double tolerance) {
    this.tolerance = Optional.of(tolerance);
    return this;
  }

  public PIDController get() {
    PIDController controller = new PIDController(kp, ki, kd);
    setpoint.ifPresent((setpoint) -> controller.setSetpoint(setpoint));
    tolerance.ifPresent((tolerance) -> controller.setTolerance(tolerance));
    return controller;
  }
}
