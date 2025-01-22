package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralHolderConstants;

public class CoralHolderSub extends SubsystemBase {

    private TalonSRX motorLeft;
    private TalonSRX motorRight;

    private double leftThrottleMultiplier = CoralHolderConstants.INVERT_MOTORS ? -1 : 1;
    private double rightThrottleMultiplier = CoralHolderConstants.INVERT_MOTORS ? 1 : -1;

    public CoralHolderSub() {
        motorLeft = new TalonSRX(CoralHolderConstants.LEFT_CAN_ID);
        motorRight = new TalonSRX(CoralHolderConstants.RIGHT_CAN_ID);
        
        var config = new TalonSRXConfiguration();
        
        motorLeft.configAllSettings(config);
        motorRight.configAllSettings(config);
        
        motorRight.setInverted(InvertType.OpposeMaster);
        motorRight.follow(motorLeft);
    }

    public void forward() {
        motorLeft.set(TalonSRXControlMode.PercentOutput, leftThrottleMultiplier * CoralHolderConstants.THROTTLE);
    }

    public void reverse() {
        motorLeft.set(TalonSRXControlMode.PercentOutput, rightThrottleMultiplier * CoralHolderConstants.THROTTLE);
    }
}
