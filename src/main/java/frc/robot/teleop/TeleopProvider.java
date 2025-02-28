package frc.robot.teleop;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;

/**
 * Provides the default command for teleop.
 */
public class TeleopProvider {
  private static Optional<TeleopProvider> inst = Optional.empty();

  private final Command disableCommand = new InstantCommand();
  private final Command teleopArcade = new DriveArcade(DriveConstants.PILOT_SETTINGS);
  private final SendableChooser<Command> chooser = new SendableChooser<>(); // pub for shuffle board

  private TeleopProvider() {
    // disabled
    disableCommand.addRequirements(Subsystems.drive);
    chooser.setDefaultOption("Disable Teleop", disableCommand);

    chooser.addOption("Arcade", teleopArcade);

    chooser.onChange(Subsystems.drive::setDefaultCommand);
    Subsystems.drive.setDefaultCommand(chooser.getSelected()); // set default on startup

    SmartDashboard.putData("Teleop Chooser", chooser);
  }

  public static TeleopProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new TeleopProvider());
    }
    return inst.get();
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}
