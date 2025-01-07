package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderSupplier implements Supplier<Encoder> {
  private final int[] port;
  private final double steps;

  private boolean invert = false;

  public EncoderSupplier(int[] port, double steps) {
    this.port = port;
    this.steps = steps;
  }

  public EncoderSupplier withInvert() {
    this.invert = true;
    return this;
  }

  public Encoder get() {
    if (port[0] < 0) {
      throw new IllegalStateException("MotorInfo : encoder port 1 < 0, check port is setup");
    }
    if (port[1] < 0) {
      throw new IllegalStateException("MotorInfo : encoder port 2 < 0, check port is setup");
    }
    if (steps < 0) {
      throw new IllegalArgumentException("MotorInfo : EncoderSteps < 0, check EncoderSteps is setup");
    }

    Encoder encoder = new Encoder(port[0], port[1], invert, Encoder.EncodingType.k2X);
    encoder.setDistancePerPulse(steps);
    encoder.setMinRate(0.1 * steps);
    encoder.setMinRate(10);
    encoder.setSamplesToAverage(5);
    return encoder;
  }
}
