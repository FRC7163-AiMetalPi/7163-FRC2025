package frc.robot.teleop;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
//import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.shufflecontrol.ShuffleTabController;
import frc.robot.utils.RangeMath.DriveBaseFit;

public class TeleopDriveSwerve extends Command {
  //private int updateShuffleCounter = 0;
  public DriveBaseFit settings;
  
  private ShuffleTabController shuffleTab;

  public TeleopDriveSwerve(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.swerveDrive);
    shuffleTab = createShuffleTab();
  }
  
  private static ShuffleTabController createShuffleTab() {
    var shuffleTab = new ShuffleTabController("Swerve Teleop");
    shuffleTab.createWidget("Drive X", BuiltInWidgets.kNumberBar,  5, 2)
      .withProperties(Map.of("min", -1, "max", 1));
    shuffleTab.createWidget("Drive Y", BuiltInWidgets.kNumberBar,  6, 3)
      .withProperties(Map.of("min", -1, "max", 1));
    shuffleTab.createWidget("Drive Rot", BuiltInWidgets.kDial,  5, 3)
      .withProperties(Map.of("min", -1, "max", 1));
    return shuffleTab;
  }

  @Override
  public void execute() {
    double limiter = OI.pilot.getRightTriggerAxis();
    double booster = OI.pilot.rightBumper().getAsBoolean()?1:0;
    // organise field relitive switch
    double[] control = settings.fitSwerve(-OI.pilot.getLeftX(), OI.pilot.getLeftY(), 
          OI.pilot.getRightX(), booster, limiter);

    var translateX = control[0];
    var translateY = control[1];
    var rotate = control[2];
    // System.out.println(control[0]+" "+control[1]+" "+control[2]+" "+control[3]+"
    // "+Subsystems.drive.getSpeedMS());
    shuffleTab.getEntry("Drive X").setDouble(translateX);
    shuffleTab.getEntry("Drive Y").setDouble(translateY);
    shuffleTab.getEntry("Drive Rot").setDouble(rotate);
    
    Subsystems.swerveDrive.drive(translateX, translateY, rotate, Variables.fieldRelative, true); // TODO fix rate limit
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
