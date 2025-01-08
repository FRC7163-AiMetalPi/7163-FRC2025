package frc.robot.teleop;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.RangeMath.DriveBaseFit;

public class TeleopDriveSwerve extends Command {
  private final DriveBaseFit settings;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);
  private final SlewRateLimiter rLimiter = new SlewRateLimiter(DriveConstants.MAX_ANGULAR_ACCELERATION);

  public TeleopDriveSwerve(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    double limiter = OI.pilot.getRightTriggerAxis();
    double booster = OI.pilot.rightBumper().getAsBoolean() ? 1 : 0;
    boolean robotRelative = OI.pilot.back().getAsBoolean();

    // organise field relative switch
    final var control = settings.fitSwerve(
        OI.pilot.getLeftX(),
        -OI.pilot.getLeftY(),
        OI.pilot.getRightX(),
        booster,
        limiter);

    var x = control[0];
    var y = control[1];
    var r = control[2];

    x = xLimiter.calculate(x) * DriveConstants.MAX_SPEED;
    y = yLimiter.calculate(y) * DriveConstants.MAX_SPEED;
    r = rLimiter.calculate(r) * DriveConstants.MAX_ANGULAR_SPEED;

    final var speeds = new ChassisSpeeds(x, y, r);
    Subsystems.drive.drive(speeds, !robotRelative);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
