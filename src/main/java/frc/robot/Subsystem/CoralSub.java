package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motorsupplier.TalonPwmMotorSupplier;

public class CoralSub extends SubsystemBase {
  private Talon motor = new TalonPwmMotorSupplier(4).get();

  public void run(double speed){
    motor.set(speed);
  }
  public void runOutTake(){
    run(0.1);
  }
  public void stopOutTake(){
    run(0);
  }
}

