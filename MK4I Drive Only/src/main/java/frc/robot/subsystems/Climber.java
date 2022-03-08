
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

TalonFX leftClimber = new TalonFX(14);
TalonFX rightClimber = new TalonFX(15);
Solenoid climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
DigitalInput limitSwitchRightDown = new DigitalInput(7);//right down
DigitalInput limitSwitchRightUp = new DigitalInput(6);//right up
DigitalInput limitSwitchLeftUp = new DigitalInput(8);//left up
DigitalInput limitSwitchLeftDown = new DigitalInput(9);//left down

  public Climber() {
    leftClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.setNeutralMode(NeutralMode.Brake);
  }
  public void climbEncoderReset(){
    leftClimber.setSelectedSensorPosition(0);
    rightClimber.setSelectedSensorPosition(0);
    
  }
  public void printLimitSwitch(){
    
    //System.out.println(limitSwitchLeftUp.get()+ "   LeftUp");
     System.out.println(limitSwitchLeftDown.get()+"   LeftDown");
    // System.out.println(limitSwitchRightUp.get()+"   RightUp");
     System.out.println(limitSwitchRightDown.get()+"   RightDown");
  }
  public boolean getLimitSwitchRightDown(){
    return limitSwitchRightDown.get();
  }
  public boolean getLimitSwitchRightUp(){
    return limitSwitchRightUp.get();
  }

  public boolean getLimitSwitchLeftUp(){
    return limitSwitchLeftUp.get();
  }

  public boolean getLimitSwitchLeftDown(){
    return limitSwitchLeftDown.get();
  }


  public void moveSolenoid(boolean solenoidPose){
    climberSolenoid.set(solenoidPose);
  }
  public void runClimber(double speed) {
    rightClimber.set(ControlMode.PercentOutput,-speed);
    leftClimber.set(ControlMode.PercentOutput,speed);
  }

  public double getLeftEncoderTicks() {
    return leftClimber.getSelectedSensorPosition();
  }

  public double getRightEncoderTicks() {
    return rightClimber.getSelectedSensorPosition();
  }

  public void runToPosition(double ticks) {
    double right = rightClimber.getSelectedSensorPosition();
    double left = leftClimber.getSelectedSensorPosition();
    System.out.println("RIGHT CLIMBER POSITION" + right);
    System.out.println("LEFT CLIMBER POSITION" + left);
    if (right > 4000 && left < -4000){
      //done

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
