package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import frc.Const;

/**
 * Hood
 */
public class Hood {

    private CANSparkMax hoodMotor = new CANSparkMax(9, CANSparkMax.MotorType.kBrushless);
    private CANDigitalInput forwardLimit = hoodMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    private CANDigitalInput reverseLimit = hoodMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    CANEncoder encoder = hoodMotor.getEncoder();

    private double zeroValue;

    public Hood() {
        //this.zeroValue = encoder.getPosition() - 23.5;
    }


    public void setPower(double power) {

        power = Math.max(Math.min(power, Const.HOOD_MAXOUTPUT), Const.HOOD_MINOUTPUT);

        hoodMotor.set(power);
        
    }

    /**
     * sets the hood position using pid and also prohibits it from exceeding max ranges
     * positive power makes the hood shrink
     */
    public void setPosition(double position) {

        double error = encoder.getPosition() - position;
        double power = Const.HOOD_Kp * error;
        System.out.println("Power: "+ power);
        setPower(-power);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    // public double getRelativePosition() {
    //     return zeroValue - encoder.getPosition();
    // }

    public void resetPotPosition() {
        System.out.println(forwardLimit.get());
        if (forwardLimit.get()) {
            encoder.setPosition(Const.HOOD_HOME_POSITION);
        }
        
    }
}