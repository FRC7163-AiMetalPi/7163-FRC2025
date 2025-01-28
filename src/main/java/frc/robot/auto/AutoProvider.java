package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;

/**
 * Provides the default command for autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final SendableChooser<Command> chooser;

  private AutoProvider() {
    chooser = new SendableChooser<>(); // pub for shuffle board

    // This is here to ensure PathPlanner is configured before we attempt to call
    // AutoBuilder.pathfindToPose, since static classes are lazily constructed.
    // I now see why WPILib's docs recommend dependency-injecting subsystems rather
    // than global static access. - Neel
    @SuppressWarnings("unused")
    final var _drive = Subsystems.drive;

    chooser.addOption("Pose Test",
        AutoBuilder.pathfindToPose(
            new Pose2d(7, 4, Rotation2d.fromDegrees(180)),
            DriveConstants.PATH_CONSTRAINTS));
    chooser.addOption("Lateral Speed Analysis", new LateralSpeedAnalysis());
    chooser.addOption("Angular Speed Analysis", new AngularSpeedAnalysis());

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  public static AutoProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new AutoProvider());
    }
    return inst.get();
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}