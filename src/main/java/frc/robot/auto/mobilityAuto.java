package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.OuttakeCommand;

public class mobilityAuto extends SequentialCommandGroup {
  public mobilityAuto(){
    addCommands(
      new ParallelRaceGroup(
        new RunCommand(() -> Subsystems.drive.drive(-1,0), Subsystems.drive),
        new WaitCommand(2)
      ),
      new InstantCommand(() -> Subsystems.drive.drive(0.0,0)),
      new OuttakeCommand()
    );
  }
 
  
}
