package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;

public class OuttakeCommand extends SequentialCommandGroup {
  public OuttakeCommand(){
    addCommands(
        new InstantCommand(() -> Subsystems.coral.reverseOutTake()),
        new WaitCommand(0.08),
        new InstantCommand(() -> Subsystems.coral.runOutTake()),
        new WaitCommand(1),
        new InstantCommand(() -> Subsystems.coral.stopOutTake())
    );
  }
  
}
