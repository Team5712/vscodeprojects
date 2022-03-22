// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {


  CANSparkMax lowerMagazine = new CANSparkMax(16, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax upperMagazine = new CANSparkMax(17, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  AnalogInput lowerBallSensor = new AnalogInput(1);
  AnalogInput upperBallSensor = new AnalogInput(0);


  
  /** Creates a new Magazine. */
  public Magazine() {
  lowerMagazine.setIdleMode(IdleMode.kBrake);
  upperMagazine.setIdleMode(IdleMode.kBrake);
  }

  public void runLowerMag(double speed) {
    lowerMagazine.set(speed);
  }

  public void runUpperMag(double speed) {
    upperMagazine.set(speed);
  }

  public double getLowerBallSensor() {
    return lowerBallSensor.getValue();
  }

  public double getUpperBallSensor() {
    //System.out.println("UPPER BALL SENSOR: " + upperBallSensor.getValue());
    return upperBallSensor.getValue();
  }

  public void stop() {
    lowerMagazine.set(0);
    upperMagazine.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
