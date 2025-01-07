package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitProgressCommand extends WaitCommand {
  private final String key;
  private final double duration;
  public WaitProgressCommand(String key, double duration) {
    super(duration);
    this.key = key;
    this.duration = duration;
  }

  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putNumber(key, m_timer.get() / duration);
  }

  @Override
  public void end(boolean interrupt) {
    super.end(interrupt);
    SmartDashboard.putNumber(key, 0);
  }
}
