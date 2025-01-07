package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

public class Range {
  private double low;
  private double high;

  public Range(double low, double high) {
    this.low = low;
    this.high = high;
  }

  public boolean contains(double number) {
    return (number >= low && number <= high);
  }

  public double clamp(double number) {
    return MathUtil.clamp(number, this.low, this.high);
  }

  @Override
  public String toString() {
    return String.format("Range(%f, %f)", this.low, this.high);
  }
}
