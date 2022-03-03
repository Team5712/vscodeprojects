// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");

  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double valid = tv.getDouble(0.0);
  double shortside = tshort.getDouble(0.0);
  double longside = tlong.getDouble(0.0);

  public Limelight() {
  }

  public double getY(){
    return y;
  }
  public double getX(){
    return x;
  }
  public double ifValidTarget(){
    return valid;
  }
  public double getShortSide(){
    return shortside;
  }
  public double getLongSide(){
    return longside;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
