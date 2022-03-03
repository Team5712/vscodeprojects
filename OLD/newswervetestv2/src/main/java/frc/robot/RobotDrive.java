
package frc.robot;

import frc.robot.Controller;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * Add your docs here.
 */
public class RobotDrive {
      //put your can Id's here!
    public static final int frontLeftDriveId = 1; 
    public static final int frontLeftCANCoderId = 9; 
    public static final int frontLeftSteerId = 2;
    //put your can Id's here!
    public static final int frontRightDriveId = 3; 
    public static final int frontRightCANCoderId = 10; 
    public static final int frontRightSteerId = 4; 
    //put your can Id's here!
    public static final int backLeftDriveId = 5; 
    public static final int backLeftCANCoderId = 11; 
    public static final int backLeftSteerId = 6;
    //put your can Id's here!   
    public static final int backRightDriveId = 7; 
    public static final int backRightCANCoderId = 12; 
    public static final int backRightSteerId = 8;   
    public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    public static PigeonIMU pigeon = new PigeonIMU(13);
    public static double [] ypr = new double[3];

    public static TalonFX frontLeftDrive = new TalonFX(frontLeftDriveId);
    public static CANCoder frontLeftCANCoder = new CANCoder(frontLeftCANCoderId);
    public static TalonFX frontLeftSteer = new TalonFX(frontLeftSteerId);

    public static TalonFX frontRightDrive = new TalonFX(frontRightDriveId);
    public static CANCoder frontRightCANCoder = new CANCoder(frontRightCANCoderId);
    public static TalonFX frontRightSteer = new TalonFX(frontRightSteerId);

    public static TalonFX backLeftDrive = new TalonFX(backLeftDriveId);
    public static CANCoder backLeftCANCoder = new CANCoder(backLeftCANCoderId);
    public static TalonFX backLeftSteer = new TalonFX(backLeftSteerId);
    
    public static TalonFX backRightDrive = new TalonFX(backRightDriveId);
    public static CANCoder backRightCANCoder = new CANCoder(backRightCANCoderId);
    public static TalonFX backRightSteer = new TalonFX(backRightSteerId);
    
    public static Controller m_controller = new Controller();
    final ShuffleboardTab tab = Shuffleboard.getTab("Encoder");
    NetworkTableEntry frontRightEntry =tab.add("frontRightangle", 0).getEntry();
    NetworkTableEntry frontLeftEntry =tab.add("frontLeftangle", 0).getEntry();
    NetworkTableEntry backLeftEntry =tab.add("backLeftangle", 0).getEntry();
    NetworkTableEntry backRightEntry =tab.add("backRightangle", 0).getEntry();
    NetworkTableEntry frontRightCANCoderEntry =tab.add("frontRightCANCoder", 0).getEntry();
    NetworkTableEntry frontLeftCANCoderEntry =tab.add("frontLeftCANCoder", 0).getEntry();
    NetworkTableEntry backLeftCANCoderEntry =tab.add("backLeftCANCoder", 0).getEntry();
    NetworkTableEntry backRightCANCoderEntry =tab.add("backRightCANCoder", 0).getEntry();
    NetworkTableEntry frontRightAbsolutePosition =tab.add("frontRightAbsolutePosition", 0).getEntry();
    NetworkTableEntry frontLeftAbsolutePosition =tab.add("frontLeftAbsolutePosition", 0).getEntry();
    NetworkTableEntry backLeftAbsolutePosition =tab.add("backLeftAbsolutePosition", 0).getEntry();
    NetworkTableEntry backRightAbsolutePosition =tab.add("backRightAbsolutePosition", 0).getEntry();
    NetworkTableEntry controllerY =tab.add("controllerY", 0).getEntry();

    
    public void configCAN(){
        backLeftCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        backRightCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        frontLeftCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        frontRightCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    }

    public double[] transformLeftInput(double fwd, double str) {
        double newFWD = fwd*Math.cos(pigeon.getFusedHeading())+str*Math.sin(pigeon.getFusedHeading());
        double newSTR = str*Math.cos(pigeon.getFusedHeading())-fwd*Math.sin(pigeon.getFusedHeading());
        double[] output = {newFWD, newSTR};
        System.out.println(pigeon.getFusedHeading());
        return output;
    }
    public double[] vectorMath(double fwd, double str, double rot) {
        double A = str - (rot * (Constants.WHEELBASELENGTH / Constants.WHEELBASEDIAMETER));
        double B = str + (rot * (Constants.WHEELBASELENGTH / Constants.WHEELBASEDIAMETER));
        double C = fwd - (rot * (Constants.WHEELBASEWIDTH / Constants.WHEELBASEDIAMETER));
        double D = fwd + (rot * (Constants.WHEELBASEWIDTH / Constants.WHEELBASEDIAMETER));
        double[] output = {A, B, C, D};
        return output;
    }
public double[] calculateSpeedAndAngle(double A, double B, double C, double D){ 
    double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2)+ Math.pow(C, 2));
    double frontLeftWheelSpeed = Math.sqrt(Math.pow(B, 2)+ Math.pow(D, 2));
    double backRightWheelSpeed = Math.sqrt(Math.pow(A, 2)+ Math.pow(D, 2));
    double backLeftWheelSpeed = Math.sqrt(Math.pow(A, 2)+ Math.pow(C, 2));
    double frontRightAngle = Math.atan2(B, C)*180/Math.PI;
    double frontLeftAngle = Math.atan2(B, D)*180/Math.PI;
    double backRightAngle = Math.atan2(A, D)*180/Math.PI;
    double backLeftAngle = Math.atan2(A, C)*180/Math.PI;
    double[] output = {
        frontRightWheelSpeed, frontLeftWheelSpeed, backRightWheelSpeed, backLeftWheelSpeed, frontRightAngle, frontLeftAngle, backRightAngle, backLeftAngle
    };
    return output;
}

public double azimuthMath(double encoderW, double wA, TalonFX motor){
    double azimuthError = encoderW - wA;
    if(Math.abs(azimuthError)>90){
        double sign = 1;
        if(azimuthError<0){
            sign = - 1;
        }
        azimuthError = azimuthError - 180 * sign;
        motor.setInverted(true);
    }
    else{
        motor.setInverted(false);
    }
    return azimuthError;
}

public double[] maxSpeed(double frontRightWheelSpeed, double frontLeftWheelSpeed, double backRightWheelSpeed, double backLeftWheelSpeed){
    double maxWheelSpeed = frontRightWheelSpeed;
    maxWheelSpeed = Math.max(maxWheelSpeed, frontLeftWheelSpeed);
    maxWheelSpeed = Math.max(maxWheelSpeed, backRightWheelSpeed);
    maxWheelSpeed = Math.max(maxWheelSpeed, backLeftWheelSpeed);
    if (maxWheelSpeed > 1){
        frontRightWheelSpeed /= maxWheelSpeed;
        frontLeftWheelSpeed /= maxWheelSpeed;
        backRightWheelSpeed /= maxWheelSpeed;
        backLeftWheelSpeed /= maxWheelSpeed;
    }

    double[] output = {
        frontRightWheelSpeed, frontLeftWheelSpeed, backRightWheelSpeed, backLeftWheelSpeed
    };
    return output;

}

public void display(){
    

}

public void drive(double str, double fwd, double rot){
    double[] transformOutput = transformLeftInput(fwd, str);
    double[] vectorOutput = vectorMath(transformOutput[0], transformOutput[1], rot);
    double[] speedandAngleOutput = calculateSpeedAndAngle(vectorOutput[0],vectorOutput[1],vectorOutput[2],vectorOutput[3]);
    double fRPID = azimuthMath(frontRightCANCoder.getAbsolutePosition(), speedandAngleOutput[4], frontRightDrive);
    double fLPID = azimuthMath(frontLeftCANCoder.getAbsolutePosition(),speedandAngleOutput[5],frontLeftDrive);
    double bRPID = azimuthMath(backRightCANCoder.getAbsolutePosition(),speedandAngleOutput[6], backRightDrive);
    double bLPID = azimuthMath(backLeftCANCoder.getAbsolutePosition(),speedandAngleOutput[7], backLeftDrive);
    double[] maxOutput = maxSpeed(speedandAngleOutput[0],speedandAngleOutput[1],speedandAngleOutput[2],speedandAngleOutput[3]);
    // System.out.println("Azimuth "+azimuthPID(fRPID));
    frontRightSteer.set(TalonFXControlMode.PercentOutput, -azimuthPID(fRPID));
    frontLeftSteer.set(TalonFXControlMode.PercentOutput, -azimuthPID(fLPID));
    backLeftSteer.set(TalonFXControlMode.PercentOutput, -azimuthPID(bLPID));
    backRightSteer.set(TalonFXControlMode.PercentOutput, -azimuthPID(bRPID));
    frontRightDrive.set(TalonFXControlMode.PercentOutput, speedandAngleOutput[0]);
    frontLeftDrive.set(TalonFXControlMode.PercentOutput, speedandAngleOutput[1]);
    backRightDrive.set(TalonFXControlMode.PercentOutput, speedandAngleOutput[2]);
    backLeftDrive.set(TalonFXControlMode.PercentOutput, speedandAngleOutput[3]);
    // System.out.println("FrontRightWheelSpeed = " + maxOutput[0]);
    // System.out.println("FrontLeftWheelSpeed = " + maxOutput[1]);
    // System.out.println("BackRightWheelSpeed = " + maxOutput[2]);
    // System.out.println("BackLeftWheelSpeed = " + maxOutput[3]);

    // System.out.println("frontRightCANCoder = " + frontRightCANCoder.getAbsolutePosition()+Constants.FRONTRIGHTOFFSET);
    // System.out.println("frontLeftCANCoder = " + frontLeftCANCoder.getAbsolutePosition()+Constants.FRONTLEFTOFFSET);
    // System.out.println("BackRightEnCoder = " + backRightCANCoder.getAbsolutePosition()+Constants.BACKRIGHTOFFSET);
    // System.out.println("backLeftCANCoder = " + backLeftCANCoder.getAbsolutePosition()+Constants.BACKLEFTOFFSET);


//     //SmartDashboard.putNumber("frontRightCANCoder = ", frontRightCANCoder.getPosition());
//     SmartDashboard.putNumber("frontLeftCANCoder = ", frontLeftCANCoder.getPosition());
//     SmartDashboard.putNumber("BackRightEnCoder = ", backRightCANCoder.getPosition());
//     SmartDashboard.putNumber("backLeftCANCoder = ", backLeftCANCoder.getPosition());
    frontRightEntry.setDouble(speedandAngleOutput[4]);
    frontLeftEntry.setDouble(speedandAngleOutput[5]);
    backRightEntry.setDouble(speedandAngleOutput[6]);
    backLeftEntry.setDouble(speedandAngleOutput[7]);
    backLeftCANCoderEntry.setDouble(backLeftCANCoder.getPosition());
    backRightCANCoderEntry.setDouble(backRightCANCoder.getPosition());
    frontRightCANCoderEntry.setDouble(frontRightCANCoder.getPosition());
    frontLeftCANCoderEntry.setDouble(frontLeftCANCoder.getPosition());
    frontRightAbsolutePosition.setDouble(frontRightCANCoder.getAbsolutePosition());
    frontLeftAbsolutePosition.setDouble(frontLeftCANCoder.getAbsolutePosition());
    backRightAbsolutePosition.setDouble(backRightCANCoder.getAbsolutePosition());
    backLeftAbsolutePosition.setDouble(backLeftCANCoder.getAbsolutePosition());
    controllerY.setDouble(m_controller.getLeftY());


//     SmartDashboard.putNumber("FrontRightAngle = ", speedandAngleOutput[4]);
//     SmartDashboard.putNumber("FrontLeftAngle = ", speedandAngleOutput[5]);
//     SmartDashboard.putNumber("BackRightAngle = ", speedandAngleOutput[6]);
//     SmartDashboard.putNumber("BackLeftAngle = ", speedandAngleOutput[7]);
 }

 public double azimuthPID(double error){
    double motorOutput = error * Constants.ANGLEKP;
    return motorOutput;
}

public double anglePID(double angle1, double POS){
    double error = angle1-POS;
    double motorOutput = error * Constants.ANGLEKP;
    return motorOutput;
}

// public void defaultState(){

//     backRightSteer.set(TalonFXControlMode.PercentOutput, anglePID(180, backRightCANCoder.getAbsolutePosition()));
//     backLeftSteer.set(TalonFXControlMode.PercentOutput, anglePID(180, backLeftCANCoder.getAbsolutePosition()));
//     frontLeftSteer.set(TalonFXControlMode.PercentOutput, anglePID(180, frontLeftCANCoder.getAbsolutePosition()));
//     frontRightSteer.set(TalonFXControlMode.PercentOutput, anglePID(180, frontRightCANCoder.getAbsolutePosition()));
    
    
// }


}
