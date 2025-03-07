package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motorsupplier.TalonPwmMotorSupplier;
import frc.robot.utils.motorsupplier.VictorMotorSupplier;

public class DriveSub extends SubsystemBase{
  private Talon leftMotor = new TalonPwmMotorSupplier(0).withInvert().get();
  private Talon rightMotor = new TalonPwmMotorSupplier(1).get();
  private WPI_VictorSPX leftMotor2 = new VictorMotorSupplier(3).withInvert().get();
  private WPI_VictorSPX rightMotor2 = new VictorMotorSupplier(4).get();
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);
  private DifferentialDrive diffDrive2 = new DifferentialDrive(leftMotor2, rightMotor2);
  
  public DriveSub(){
    diffDrive.setSafetyEnabled(false);
    diffDrive2.setSafetyEnabled(false);
    leftMotor.setSafetyEnabled(false);
    rightMotor.setSafetyEnabled(false);
    leftMotor2.setSafetyEnabled(false);
    rightMotor2.setSafetyEnabled(false);
  }

  public void drive(double speed, double turn){
    diffDrive.arcadeDrive(speed, turn);
    diffDrive2.arcadeDrive(speed, turn);
  }
}
