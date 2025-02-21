package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motorsupplier.TalonPwmMotorSupplier;

public class DriveSub extends SubsystemBase{
  private Talon leftMotor = new TalonPwmMotorSupplier(0).get();
  private Talon rightMotor = new TalonPwmMotorSupplier(1).get();
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  public void drive(double speed, double turn){
    diffDrive.arcadeDrive(speed, turn);
  }
}
