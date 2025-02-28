package frc.robot.teleop;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.utils.SpeedRateLimiter;
import frc.robot.utils.RangeMath.DriveBaseFit;

public class DriveArcade extends Command{
  private final SlewRateLimiter speedLimiter = new SpeedRateLimiter(1, -2, 0);

  private final DriveBaseFit settings;

  public DriveArcade(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    double limiter = OI.pilot.getRightTriggerAxis();
    double booster = OI.pilot.getHID().getRightBumperButton() ? 1 : 0;
    double speed = OI.pilot.getLeftY();
    double steer = -OI.pilot.getRightX();
    final var control = settings.fitTank(
      speed, steer, booster, limiter
    );

    speed = control[0];
    steer = control[2];
    speed = speedLimiter.calculate(speed);

    Subsystems.drive.drive(speed, steer);
  }
  
}
