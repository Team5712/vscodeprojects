/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.ObjectInputStream.GetField;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.plugins.tiff.GeoTIFFTagSet;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.control.NetworkTableServer;
import frc.robot.control.PID;
import frc.robot.pathfinding.AStar;
import frc.robot.pathfinding.Node;
import frc.robot.util.Const;
import frc.robot.util.RoboMath;
import frc.robot.util.Values;
import frc.robot.vision.VisionProcessing;

public class Robot extends IterativeRobot {

    // DRIVE OBJECTS
    // private CANSparkMax leftMaster = new CANSparkMax(2, MotorType.kBrushless);
    // private CANSparkMax rightMaster = new CANSparkMax(5, MotorType.kBrushless);

    // private CANSparkMax leftSlave1 = new CANSparkMax(3, MotorType.kBrushless);
    // private CANSparkMax rightSlave1 = new CANSparkMax(6, MotorType.kBrushless);

    // private CANSparkMax leftSlave2 = new CANSparkMax(1, MotorType.kBrushless);
    // private CANSparkMax rightSlave2 = new CANSparkMax(4, MotorType.kBrushless);

    // private CANEncoder leftEncoder;
    // private CANEncoder rightEncoder;

    private WPI_TalonSRX leftMaster = new WPI_TalonSRX (2);
    private WPI_TalonSRX rightMaster = new WPI_TalonSRX (5);


    private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

    // CONTROLLERS
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);

    NetworkTableServer networkTableServer;

    AHRS gyro = new AHRS(Port.kMXP);

    Timer timer;

    private double xPosition;
    private double yPosition;

    private double angle;

    PID turnpid;
    PID drivepid;

    RoboMath RoboMath;

    NetworkTableInstance instance;
    NetworkTable table;
    NetworkTableEntry clickX;

    List<Node> drivePoints;

    @Override
    public void robotInit() {

        timer = new Timer();

        RoboMath = new RoboMath();

        gyro.zeroYaw();

        // leftEncoder = leftMaster.getEncoder();
        // rightEncoder = rightMaster.getEncoder();

        // leftEncoder.setPosition(0);
        // rightEncoder.setPosition(0);

        Values.init();

        turnpid = new PID(.025, 0, 0, 0, 0);
        drivepid = new PID(.05, 0, 0, 0, 0);

        // leftSlave1.follow(leftMaster);
        // leftSlave2.follow(leftMaster);

        // rightSlave1.follow(rightMaster);
        // rightSlave2.follow(rightMaster);
    }

    /**
     * @return the angle
     */
    public double getAngle() {
        return angle;
    }

    /**
     * @param angle the angle to set
     */
    public void setAngle(double angle) {
        this.angle = angle;
    }

    /**
     * @return the yPosition
     */
    public double getyPosition() {
        return yPosition;
    }

    /**
     * @param yPosition the yPosition to set
     */
    public void setyPosition(double yPosition) {
        this.yPosition = yPosition;
    }

    /**
     * @return the xPosition
     */
    public double getxPosition() {
        return xPosition;
    }

    /**
     * @param xPosition the xPosition to set
     */
    public void setxPosition(double xPosition) {
        this.xPosition = xPosition;
    }

    @Override
    public void teleopInit() {

        leftMaster.setInverted(false);
        rightMaster.setInverted(false);
        

        gyro.zeroYaw();
        timer.start();

    }

    @Override
    public void teleopPeriodic() {

        // drive.tankDrive(leftJoystick.getY(), rightJoystick.getY());

        System.out.println("ratio " + Values.getNumber("TICK_TO_INCH_RATIO"));

        this.updatePosition();
    }

    @Override
    public void autonomousInit() {

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        gyro.zeroYaw();
    }

    @Override
    public void autonomousPeriodic() {

    }

    /**
     * will return motor output to spin on a dime using PID
     * 
     * @param setpoint
     * @return
     */
    public double getTurnPIDOutput(double setpoint) {

        // error is the distance from where we want to go from where we are now
        // double error = RoboMath.getAngleToDegree(setpoint,
        // RoboMath.toUnitCircleDegrees(gyro.getYaw()));
        double error = setpoint;
        // calculate proportion value
        double p = turnpid.getP() * error;

        // i_zone for perfecting distance to target
        if (Math.abs(error) <= turnpid.getI_zone() || turnpid.getI_zone() == 0.0f) {
            turnpid.setI_state(turnpid.getI_state() + (error * turnpid.getI()));
        } else {
            turnpid.setI_state(0);
        }
        double d = (error - turnpid.getPrev_err());
        turnpid.setPrev_err(error);
        d *= turnpid.getD();

        // static feed forward value based on how far we need to go
        double f = error * turnpid.getF();

        // add up all of our values for our output
        double output = p + d + turnpid.getI_state();
        return output;
    }

    /**
     * will return the motor output for a given setpoint using PID
     * 
     * @param setpoint
     * @return
     */
    public double getDrivePIDOutput(double setpoint) {

        // error is the distance from where we want to go from where we are now
        double err1 = setpoint;
        // calculate proportion value
        double p1 = drivepid.getP() * err1;

        if (Math.abs(err1) <= drivepid.getI_zone() || drivepid.getI_zone() == 0.0f) {
            drivepid.setI_state(drivepid.getI_state() + (err1 * drivepid.getI()));
        } else {
            drivepid.setI_state(0);
        }

        double d1 = (err1 - drivepid.getPrev_err());
        drivepid.setPrev_err(err1);
        ;
        d1 *= drivepid.getD();

        // static feed forward value based on how far we need to go
        double f1 = err1 * drivepid.getF();

        // add up all of our values for our output
        double driveOutput = p1;

        double output = driveOutput;
        // make sure the output is not greater than our max or less than our min
        // System.out.println(output);
        // double final_output = fminf(fmaxf(output, pid.Kminoutput), pid.Kmaxoutput);;
        return output;

        // set power
    }

    @Override
    public void testPeriodic() {
    }


    /**
     * runs some calculations using RoboMath and updates the robots current position
     * this must be called on every iteration to update the position!
     */
    public void updatePosition() {
        // Gets current x and y positions
        double coordinates[] = RoboMath.calculateCoordinatePosition(getxPosition(), getyPosition(),
                leftEncoder.getPosition(), rightEncoder.getPosition(), gyro.getYaw());
        this.setxPosition(coordinates[0]);
        this.setyPosition(coordinates[1]);
        gyro.getYaw();

        this.setAngle(RoboMath.toUnitCircleDegrees(gyro.getYaw()));
    }

    public double getDistance(double coordinate[]) {
        double[] points = { coordinate[0], coordinate[1] };
        double[] getError = RoboMath.calculateRelativeAngle(this.getxPosition(), this.getyPosition(),
                RoboMath.toUnitCircleDegrees(gyro.getYaw()), points);
        return getError[0];
    }

    /**
     * drive to each point in the given array of [x, y] pairs
     * 
     * @param coordinateSequence
     */
    public double[] driveToPoints(double coordinateSequence[][]) {
        double[] output = null;

        // if (coordinateSequence.length != 1) {
        // output = driveToPoint(coordinateSequence[1]);
        // } else {
        // output = driveToPoint(coordinateSequence[0]);
        // }

        output = driveToPoint(coordinateSequence[coordinateSequence.length - 1]);

        return output;
    }

    /**
     * drive to each point in the given array of [x, y] pairs
     * 
     * @param coordinateSequence
     */
    public double[] driveToPoints(List<Node> coordinateSequence) {
        double[] output;

        if (coordinateSequence.size() != 1) {
            output = driveToPoint(coordinateSequence.get(1).toDoubleArray());
        } else {
            output = driveToPoint(coordinateSequence.get(0).toDoubleArray());
        }

        return output;
    }

    /**
     * turn towards the given point if it's outside of our threshold range and drive
     * towards it once we are within it. This is effectively a "point and shoot"
     * method of iterating our points
     * 
     * TODO: smooth out our driving process by removing the threshold
     * 
     * This method derives the angle to turn from RoboMath.calulateRelativeAngle
     * 
     * @param point
     * @return
     */
    public double[] driveToPoint(double[] point) {

        double[] getError = RoboMath.calculateRelativeAngle(this.getxPosition(), this.getyPosition(),
                RoboMath.toUnitCircleDegrees(gyro.getYaw()), point);
        double angleError = getError[1];
        double distanceError = getError[0];

        // System.out.println(distanceError + " distance Error");

        // Calulates the speed it needs to turn at and drive speed using PID
        double turnSpeed = this.getTurnPIDOutput(angleError);
        double driveSpeed = this.getDrivePIDOutput(distanceError);

        // System.out.println(turnSpeed + ":turnSpeed " + driveSpeed + ":driveSpeed");

        // Checks to see if the angle error is with inside the tolerance degrees if
        // not it will turn then continue to drive forwards
        double tolerance = 15;

        if (driveSpeed > 1)
            driveSpeed = 1;
        if (driveSpeed < -1)
            driveSpeed = -1;

        double leftDrive;
        double rightDrive;

        if (Math.abs(angleError) < tolerance && Math.abs(distanceError) > 2) {
            rightDrive = (driveSpeed * Const.PATHFINDING_SPEED + turnSpeed);
            leftDrive = (driveSpeed * Const.PATHFINDING_SPEED - turnSpeed);
        } else {
            rightDrive = turnSpeed;
            leftDrive = -turnSpeed;
        }

        // System.out.println(leftDrive + ":leftDrive " + rightDrive + ":rightDrive");
        return new double[] { leftDrive, rightDrive };
    }
}