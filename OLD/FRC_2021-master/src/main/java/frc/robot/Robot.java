/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Const;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */

    public Joystick leftJoystick = new Joystick(0);
    public Joystick rightJoystick = new Joystick(1);
    public Joystick auxJoystick = new Joystick(2);
    public String autonMode = "";

    private Vision vision = new Vision();
    private Climber climber = new Climber();
    private Intake intake = new Intake();
    private Hood hood = new Hood();

    private RobotContainer container;

    private Turret turret = new Turret();

    private Timer autoTimer = new Timer();
    private Timer IRSensorTimer = new Timer();
    private Timer conveyorReverseTimer = new Timer();

    boolean isIntaking = false;
    boolean isColorSensorActive = true;

    private boolean isConveyorReversed = true;
    private boolean isIRConveyorRunning = false;
    private DigitalInput IRSensor = new DigitalInput(0);
    private AnalogInput PESwitch = new AnalogInput(0);
    public int commandNumber = 0;

    public boolean isball = false;


    @Override
    public void robotInit() {
        container = new RobotContainer();
        container.drive.configBrakeMode(true);
        autoTimer.reset();
    }

    @Override
    public void autonomousInit() {

        // Command autonomousCommand = container.getAutonomousCommand();

        // if (autonomousCommand != null) {
        //     autonomousCommand.schedule();
        // }

        // container.drive.configBrakeMode(false);

        container.resetSensors();
        System.out.println("Autonomous has started");
        autoTimer.start();

        
    }

    @Override
    public void autonomousPeriodic() {
        // //intakeSensor();
        // //System.out.println(container.drive.getWheelSpeeds());
        // CommandScheduler.getInstance().run();
        // if (commandNumber == 0) {
        //     CommandScheduler.getInstance().run();
        //     System.out.println("RUNNING FIRST");
        // } else if (commandNumber == 1) {
        // // CommandScheduler.getInstance().run();

        //     System.out.println("ONTO SECOND COMMAND");
        // } else {
        //     System.out.println(" no work :(");
        switch (commandNumber) {
        case 0:

            if (autoTimer.get() > 10) {
                turret.stop();
                commandNumber++;
            } else {

                if (autoTimer.get() > 3) {
                    intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
                    intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
                    intake.setIntakePower(-Const.INTAKE_SPEED);
                }

                turret.shoot();
            }

            break;

        case 1:

            if (autoTimer.get() > 12) {
                commandNumber++;
            } else {
                turret.stop();
                container.drive.TankDrive(-0.55, 0.55);
            }

            break;

        case 2:
            intake.setIntakePower(0);
            intake.setFrontConveyorPower(0);
            intake.setBackConveyorPower(0);

            container.TankDrive(0, 0);
            turret.stop();

            break;

        default:
            System.out.println("invalid command number: " + commandNumber);
        }
    }
    

    @Override
    public void teleopInit() {
        isIntaking = false;
        container.resetSensors();
        turret.stop();
        intake.setIntakePower(0);
        intake.setBackConveyorPower(0);
        intake.setFrontConveyorPower(0);
        hood.resetPotPosition();
        container.drive.configBrakeMode(false);
        // turret.resetShooterTicks();
    }

    @Override
    public void teleopPeriodic() {
        handlePrimaryDriverInput();
        handleSecondaryDriverInput();
        System.out.println("NEW SENSOR: " + IRSensor.get());
        
        //container.printGyroYaw();
        
    }

    public void diabledPeriodic() {
        container.drive.configBrakeMode(true);
    }

    /**
     * Handle input for the primary driver ((left/right)Joystick)
     */
    private void handlePrimaryDriverInput() {
        //Print Hood Angle
        //System.out.println("Hood Angle: " + hood.getPosition());


        // Shifting
        if (leftJoystick.getRawButton(1)) {
            container.shift(true);
        } else {
            container.shift(false);
        }

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Intake
        // .............................................................................

        //run intake in and out
        boolean isball = (PESwitch.getValue() > Const.PESWITCH_UPPER_LIMIT);
        if (rightJoystick.getRawButtonPressed(2)) {
            isIntaking = !isIntaking;
        }

        if (isIntaking) {
            intake.setIntakePower(-Const.INTAKE_SPEED);
            intake.setSolenoid(true);
        }
        else {
            intake.setIntakePower(0);
            intake.setSolenoid(false);
        }

        // left joystick middle button reverse WHILE intaking
        if (rightJoystick.getRawButton(2)) {
            intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
            intake.setIntakePower(Const.INTAKE_SPEED);
            intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
        } else if (leftJoystick.getRawButton(3)) {
            intake.setFrontConveyorPower(-Const.FRONT_CONVEYOR_SPEED);
        } else if (isball && isColorSensorActive && isIntaking) {
            IRSensorTimer.start();
            isIRConveyorRunning = true;
        } else {
            intake.setFrontConveyorPower(0);
        }

        if (isIRConveyorRunning && IRSensorTimer.get() < .5) {
            intake.setFrontConveyorPower(0.6);
        } else if (!leftJoystick.getRawButton(3) && !leftJoystick.getRawButton(4)) {
            isIRConveyorRunning = false;
            intake.setFrontConveyorPower(0);
            IRSensorTimer.reset();
        }

        //ADDED BACK SENSOR. TODO: TEST SENSOR
        // if (isIRConveyorRunning && IRSensorTimer.get() < .5 && IRSensor.get()) {
        //     intake.setFrontConveyorPower(0.6);
        // } else if (!leftJoystick.getRawButton(3) && !leftJoystick.getRawButton(4)) {
        //     isIRConveyorRunning = false;
        //     intake.setFrontConveyorPower(0);
        //     IRSensorTimer.reset();
        // }

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Drive Handling
        // .............................................................................

        // Auto center on target - I added an "or" condition so 2nd driver could control - Chris
        if (rightJoystick.getRawButton(1) || auxJoystick.getRawButton(2)) {

            double[] turnValues = vision.getTurnValues();
            //System.out.println("left adjust " + turnValues[0] + "right adjust " + turnValues[1]);
            double turretGoal = 0;
            double turretError = turretGoal - turnValues[0];
            if (turnValues[0] > 0) {
                //more power when turret needs to turn right
                double finalTurretOutput = (turretError * Const.TURRET_TURN_RIGHT_Kp) + Const.TURRET_Ki;
                turret.setTurretYawPower(finalTurretOutput);
            } else {
                //less power when turret needs to turn left
                double finalTurretOutput = (turretError * Const.TURRET_TURN_LEFT_Kp) + Const.TURRET_Ki;
                turret.setTurretYawPower(finalTurretOutput);
            }
            
                //Turret Motor - Chris
            
                //Turret Motor - Chris
                
            container.TankDrive(-leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1));

            //container.TankDrive(turnValues[0] - leftJoystick.getRawAxis(1),
                    //turnValues[1] - rightJoystick.getRawAxis(1));

            // default drive
        } 
        
        else if (auxJoystick.getRawButton(5) || leftJoystick.getRawButton(4)) {
            turret.setTurretYawPower(0.20);
            // Left Bumper for manual left turret yaw - Chris
            container.TankDrive(-leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1));
        }
        else if (auxJoystick.getRawButton(6) || rightJoystick.getRawButton(4)) {
            turret.setTurretYawPower(-0.25);
            // Right Bumper for manual right turret yaw - Chris
            container.TankDrive(-leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1));
        }
        else {
            // TODO: bring this back
            //Vision.disableLEDS();
             container.TankDrive(-leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1));
             turret.setTurretYawPower(0);

             //Drive should really be moved outside this case statement since
             //it's no longer used for targeting - Chris
        }
    }

    /**
     * Handle input for the secondary driver (auxJoystick)
     */
    private void handleSecondaryDriverInput() {
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Turret
        // .............................................................................
        //System.out.println(hood.getPosition());
        // A button hood back
        if (auxJoystick.getRawButton(1)) {
            hood.setPower(.1);
            //System.out.println(hood.getPosition());
            // Y button hood forward
        } else if (auxJoystick.getRawButton(4)) {
            hood.setPower(-.1);
            // X button reset hood to home position. Potentiometer value reset upon hitting limit switch
        } else if (auxJoystick.getRawButton(3)) {
            //Vision.setNumber("pipeline", 0);
            Vision.enableLEDS();
            hood.resetPotPosition();
            hood.setPower(.1);
            // Hold B button for shooting on the line
        } else if (auxJoystick.getRawButton(2) || rightJoystick.getRawButton(1)) {
            Vision.setNumber("pipeline", 0);
            Vision.enableLEDS();
            hood.setPosition(Vision.getDesiredHoodAngleToTarget());
            // hood.resetPotPosition();
            //System.out.println("Actual Angle: "+ hood.getPosition());
            //System.out.println("Desired Angle: "+ Vision.getDesiredHoodAngleToTarget());

        } else if (auxJoystick.getRawButton(7)) {
            Vision.enableLEDS();
            hood.setPosition(Const.HOOD_CONTROL_PANEL_POSITION);
        } else {
            hood.setPower(0);
        }


            //chris
        if (!leftJoystick.getRawButton(3)) {

            if (auxJoystick.getRawAxis(3) > .3) {
                turret.shoot();
                if (turret.getShooterError() < Math.abs(Const.INTAKE_BACK_CONVEYOR_THRESHOLD)) {
                    // TODO: debug this
                    intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
                    intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
                }

            
            } else {
                turret.stop();
                intake.setBackConveyorPower(0);
            }
            //chris
        } else {
            intake.setBackConveyorPower(-0.8);
        }

        
        // if (auxJoystick.getRawButtonPressed(5)) {
        //     isConveyorReversed = true;
        //     conveyorReverseTimer.start();
        // }

        // if (isConveyorReversed && conveyorReverseTimer.get() < 0.1 && auxJoystick.getRawButton(5)) {
        //     System.out.println("running timer " + IRSensorTimer.get());
        //     intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
        //     intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
        // } else if (!isConveyorReversed && auxJoystick.getRawButton(5)) {
        //     intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
        //     intake.setBackConveyorPower(Const.BACK_CONVEYOR_SPEED);
        // } else if (auxJoystick.getRawButton(6)) {
        //     intake.setFrontConveyorPower(Const.FRONT_CONVEYOR_SPEED);
        // } else {
        //     isConveyorReversed = false;
        //     conveyorReverseTimer.reset();
        // }

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Climbing
        // .............................................................................

        // right joystick handles climbing
        if (auxJoystick.getRawAxis(5) > 0.3) {
            climber.setPower(-Const.CLIMB_SPEED);
        } else if (auxJoystick.getRawAxis(5) < -0.3) {
            climber.setPower(Const.CLIMB_SPEED);
        } else {
            climber.setPower(0);
        }

    }

    public void intakeSensor() {
        intake.setIntakePower(-Const.INTAKE_SPEED);

        intake.setSolenoid(true);
        intake.setFrontConveyorPower(0.5);

        // (red > Const.COLOR_RED_THRESHOLD && green > Const.COLOR_GREEN_THRESHOLD)
        // if ( isColorSensorActive) {
            
        //     IRSensorTimer.start();
        //     isIRConveyorRunning = true;
        // } else {
        //     intake.setFrontConveyorPower(0);
        //     // System.out.println("red " + colorSensor.getRed() + " green " +
        //     // colorSensor.getGreen() + " alpha " + alpha);
        // }

        // if (isIRConveyorRunning && IRSensorTimer.get() < 0.5) {
        //     // System.out.println("running timer " + IRSensorTimer.get());
        //     intake.setFrontConveyorPower(-0.5);
        // } else if (!leftJoystick.getRawButton(3) && !leftJoystick.getRawButton(4)) {
        //     isIRConveyorRunning = false;
        //     intake.setFrontConveyorPower(0);
        // }
    }

}
