
package frc.robot;
import frc.robot.Joystick;

import java.lang.Math;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;



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
    
    public static Joystick m_controller = new Joystick();

    public double[] transformLeftInput(double fwd, double str) {
        double newFWD = fwd*Math.cos(0)+str*Math.sin(0);
        double newSTR = str*Math.cos(0)-fwd*Math.sin(0);
        double[] output = {newFWD, newSTR};
        return output;
    }
    public double[] vectorMath(double fwd, double str, double rot) {
        double A = str - rot * (Constants.WHEELBASELENGTH / Constants.WHEELBASEDIAMETER);
        double B = str + rot * (Constants.WHEELBASELENGTH / Constants.WHEELBASEDIAMETER);
        double C = fwd - rot * (Constants.WHEELBASEWIDTH / Constants.WHEELBASEDIAMETER);
        double D = fwd + rot * (Constants.WHEELBASEWIDTH / Constants.WHEELBASEDIAMETER);
        double[] output = {A, B, C, D};
        return output;
    }
public double[] calculateSpeedAndAngle(double A, double B, double C, double D){ 
    double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2)+ Math.pow(C, 2));
    double frontLeftWheelSpeed = Math.sqrt(Math.pow(B, 2)+ Math.pow(D, 2));
    double backRightWheelSpeed = Math.sqrt(Math.pow(A, 2)+ Math.pow(C, 2));
    double backLeftWheelSpeed = Math.sqrt(Math.pow(A, 2)+ Math.pow(D, 2));
    double frontRightAngle = Math.atan2(B, C);
    double frontLeftAngle = Math.atan2(B, D);
    double backRightAngle = Math.atan2(A, C);
    double backLeftAngle = Math.atan2(D, D);
    double[] output = {
        frontRightWheelSpeed, frontLeftWheelSpeed, backRightWheelSpeed, backLeftWheelSpeed, frontRightAngle, frontLeftAngle, backRightAngle, backLeftAngle
    };
    return output;
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
public void drive(double str, double fwd, double rot){
    double[] transformOutput = transformLeftInput(fwd, str);
    double[] vectorOutput = vectorMath(transformOutput[0], transformOutput[1], rot);
    double[] speedandAngleOutput = calculateSpeedAndAngle(vectorOutput[0],vectorOutput[1],vectorOutput[2],vectorOutput[3]);
    double[] maxOutput = maxSpeed(speedandAngleOutput[0],speedandAngleOutput[1],speedandAngleOutput[2],speedandAngleOutput[3]);

System.out.println("FrontRightWheelSpeed = " + maxOutput[0]);
System.out.println("FrontLeftWheelSpeed = " + maxOutput[1]);
System.out.println("BackRightWheelSpeed = " + maxOutput[2]);
System.out.println("BackLeftWheelSpeed = " + maxOutput[3]);
System.out.println("FrontRightAngle" + speedandAngleOutput[4]);
System.out.println("FrontLeftAngle" + speedandAngleOutput[5]);
System.out.println("BackRightAngle" + speedandAngleOutput[6]);
System.out.println("BackLeftAngle" + speedandAngleOutput[7]);
}
}
