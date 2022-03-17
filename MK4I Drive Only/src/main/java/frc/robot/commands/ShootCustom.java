// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShootCustom extends CommandBase {
  /** Creates a new ShootHigh. */

  private Shooter m_shooter;
  private Magazine m_magazine;
  private double m_RPM;
  private long endTimer;

  public ShootCustom(Shooter shooter, Magazine magazine, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_magazine = magazine;
    m_RPM = RPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.customShootHigh(m_RPM);
    
    // try {
    //   Thread.sleep(2000);
    // } catch (InterruptedException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    //System.out.println(m_shooter.leftSpeed());
    if (System.currentTimeMillis() - endTimer > 2000) {
      if (m_shooter.leftSpeed() < m_RPM*1.1 && m_shooter.leftSpeed() > m_RPM*0.9) {
        if(m_magazine.getUpperBallSensor() < Constants.UPPER_BALL_SENSOR_THRESHOLD){
          m_magazine.runLowerMag(.2);
          m_magazine.runUpperMag(-.2);
        }
        else{
          m_magazine.runLowerMag(0);
          m_magazine.runUpperMag(-.2); 
        }
  
      }
      else{
        m_magazine.runLowerMag(0);
        m_magazine.runUpperMag(0);   
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_magazine.runLowerMag(0);
    m_magazine.runUpperMag(0);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
