// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LEDs.FlashSolidLEDCommand;
import frc.robot.LEDs.RepeatedFlashLEDCommand;
import frc.robot.LEDs.SolidLEDCommand;
import frc.robot.auto.AutoProvider;
import frc.robot.teleop.TeleopProvider;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * Subsystemsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoProvider autoProvider = AutoProvider.getInstance();
  private final TeleopProvider teleopProvider = TeleopProvider.getInstance();

  /**
   * The container for the robot. Contains Subsystemsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Robot Automations
    // flash leds yellow during endgame
    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30)
        .onTrue(new RepeatedFlashLEDCommand((FlashSolidLEDCommand)(new FlashSolidLEDCommand(Color.kYellow, 300).withZone()), 5));

    DigitalInput dio0 = new DigitalInput(0);
    new Trigger(()->dio0.get()).whileTrue(new SolidLEDCommand(Color.kGreen).withZone(1));
    // +----------------+
    // | PILOT CONTROLS |
    // +----------------+

    // --- Manual Controls ---

    // Invert Drive
    // OI.pilot.start().onTrue(new InstantCommand(() ->
    // Variables.invertDriveDirection = !Variables.invertDriveDirection));

    //OI.pilot.y().onTrue(new InstantCommand(()->BatteryPercentLEDCommand.runFor(50)));
    OI.pilot.a().onTrue(new FlashSolidLEDCommand(Color.kCrimson, 1000).withZone());
    OI.pilot.b().onTrue(new RepeatedFlashLEDCommand(
        (FlashSolidLEDCommand)(new FlashSolidLEDCommand(Color.kYellow, 200).withZone(new int[]{1,2})),
        5).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    OI.pilot.x().onTrue(new RepeatedFlashLEDCommand(
        (FlashSolidLEDCommand)(new FlashSolidLEDCommand(Color.kBlue, 200).withZone(new int[]{0})),
        5));

    // set field relitive  Arrays.asList(Color.kBlue, Color.kRed)
    OI.pilot.leftTrigger(0.5)
        .onTrue(new InstantCommand(() -> Variables.fieldRelative = false))
        .onFalse(new InstantCommand(()-> Variables.fieldRelative = true));

    //OI.pilot.start()
    //    .onTrue(
    //        new InstantCommand(() -> Subsystems.swerveDrive.zeroHeading(), Subsystems.swerveDrive));

    // Drive bindings handled in teleop command
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return teleopProvider.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoProvider.getSelected();
  }
}
