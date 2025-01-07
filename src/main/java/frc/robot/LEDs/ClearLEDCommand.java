package frc.robot.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public class ClearLEDCommand extends SolidLEDCommand {
  public ClearLEDCommand() {
    super(Color.kBlack);
  }
  
  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
