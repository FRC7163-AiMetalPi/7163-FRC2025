package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.OuttakeCommand;

public class mobiltyAuto extends SequentialCommandGroup {
  public mobiltyAuto(){
    addCommands(
      new ParallelRaceGroup(
        new InstantCommand(() -> Subsystems.drive.drive(1,0)).repeatedly(),
        new WaitCommand(10)
      ),
      new InstantCommand(() -> Subsystems.drive.drive(0.0,0)),
      new OuttakeCommand()
    );
  }
 
  
}
