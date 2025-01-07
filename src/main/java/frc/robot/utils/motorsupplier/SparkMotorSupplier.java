package frc.robot.utils.motorsupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMotorSupplier extends MotorSupplier<SparkMax> {

  public SparkMotorSupplier(int port) {
    super(port);
  }

  @Override
  public MotorSupplier<SparkMax> withBrake() {
    throw new UnsupportedOperationException();
  }

  @Override
  public MotorSupplier<SparkMax> withSafety() {
    throw new UnsupportedOperationException();
  }

  public SparkMax get() {
    if (port < 0) {
      System.out.println("MotorInfo : motor port num < 0, check port is defined : " + port);
      return new SparkMax(port, MotorType.kBrushed);
    }

    final var spark = new SparkMax(port, MotorType.kBrushed);
    final var config = new SparkMaxConfig();
    config.inverted(invert);

    if (voltageComp) {
      config.voltageCompensation(12);
    } else {
      config.disableVoltageCompensation();
    }

    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    return spark;
  }
}
