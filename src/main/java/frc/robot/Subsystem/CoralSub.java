package frc.robot.Subsystem;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motorsupplier.SparkMotorSupplier;

public class CoralSub extends SubsystemBase {
  private SparkMax motor = new SparkMotorSupplier(1).get();

  public void run(double speed){
    motor.set(speed);
  }
  public void runOutTake(){
    run(0.5);
  }
  public void reverseOutTake(){
    run(-0.7);
  }
  public void stopOutTake(){
    run(0);
  }
}

