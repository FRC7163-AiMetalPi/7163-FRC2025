package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class SpeedRateLimiter extends SlewRateLimiter {
  
  public SpeedRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue){
    super(positiveRateLimit, negativeRateLimit, initialValue);
  }

  @Override
  public double calculate(double input){
    return Math.copySign(super.calculate(Math.abs(input)), input);
  }

}
