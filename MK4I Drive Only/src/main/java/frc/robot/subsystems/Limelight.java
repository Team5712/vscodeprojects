// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  double kP = .02;
  double kI = .05;
  double kD = .02;
  double integral, previous_error = 0;
  double output;
  Boolean targetingOn = false;

  public Limelight() {
    SmartDashboard.putBoolean("DISTANCE", tarmacDistance());
    SmartDashboard.putBoolean("ANGLE", alignGood());
  }

  public double getY(){
    return y;
  }
  public double getX(){
    return tx.getDouble(0.0);
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

  public boolean tarmacDistance() {
    if (ty.getDouble(0.0) > 6 && ty.getDouble(0.0) < 10) {
      return true;
    } else {
      return false;
    }
  }

  public boolean alignGood() {
    if (tx.getDouble(0.0) > -1 && tx.getDouble(0.0) < 1) {
      return true;
    } else {
      return false;
    }
  }
  
  public void targetingOnFunc(){
    targetingOn = true;
  }
  public void targetToggle(boolean onOff) {
    targetingOn = onOff;
  }

  public double turnToTarget() {
    //double x = getX();
    //System.out.println(getX());
    if (targetingOn) {
      //System.out.println(getX());
      // if (getX() > 1){
      //   if (getX() > 5){
      //     output = -.04 * getX();
      //   }
      //   else{
      //     output = -.02 * getX();
      //   }  
      // } 
      // if (getX() < -1){
      //   if (getX() < -5){
      //     output = -.04 * getX();
      //   }
      //   else{
      //     output = -.02 * getX();
      //   }  
      // } 
      // if(Math.abs(getX()*-.04)>.5){
      //   output = Math.signum(getX())*-.5;
      // }
      double error = 0 - getX();
      this.integral += (error * 0.02);
      double derivative = (error - this.previous_error);
      double rcw = kP * error + kI * this.integral + kD * derivative;
      previous_error = error;
      if(Math.abs(error) < 0.25) {
        integral = 0;
      }

      return Math.max(Math.min(rcw, .5), -.5);
    }
    return 0;
  }

  public double[] calcHoodAndRPM() {
    System.out.println("LIMELIGHT" + ty.getDouble(0.0));
    //ty rpm hood
    double arr[][] = { { -11.85, 15000, -9.5 },
        { -11, 14750, -9.4 },
        { -9.57, 14500, -9.2 },
        { -8.34, 14000, -9 },
        { -6.7, 13750, -8.5 },
        { -5.12, 13400, -8 },
        { -4, 13000, -6 },
        { -3.34, 13200, -7.25 },
        { -1.90, 13000, -6.75 },
        { 0.37, 12225, -5.5 },
        { 2.78, 12000, -5 },
        { 5.6, 11600, -4.5 },
        { 8.89, 11400, -4 },
        { 10.02, 11000, -2 }
    };

    int start = 0, end = arr.length - 1;
    // ty is limelight value
    double yoffset = ty.getDouble(0.0);
    int ans = 0;
    while (start <= end) {
      int mid = (start + end) / 2;

      if (arr[mid][0] < yoffset) {
        start = mid + 1;
      }

      else {
        ans = mid;
        end = mid - 1;
      }
    }
    System.out.println("Shooter speed is " + arr[ans][1]);
    System.out.println("Shooter angle is is " + arr[ans][2]);
    System.out.println("LIMELIGHT" + ty.getDouble(0.0));
    double[] output = {arr[ans][1], arr[ans][2]};
    return output;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("DISTANCE", tarmacDistance());
  }
}
