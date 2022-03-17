// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShootWithWaitCustom extends CommandBase {
  /** Creates a new ShootHigh. */

  private Shooter m_shooter;
  private Magazine m_magazine;
  private double m_RPM;
  enum State { SHOOT1, WAIT, SHOOT2 };
  State state;
  private double waitStarted;

  public ShootWithWaitCustom(Shooter shooter, Magazine magazine, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_magazine = magazine;
    m_RPM = RPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.SHOOT1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.customShootHigh(m_RPM);
    // //System.out.println(m_shooter.leftSpeed());
    if (m_shooter.leftSpeed() < m_RPM*1.1 && m_shooter.leftSpeed() > m_RPM*0.9) {
      if(m_magazine.getUpperBallSensor() < Constants.UPPER_BALL_SENSOR_THRESHOLD){
        m_magazine.runLowerMag(0);
        m_magazine.runUpperMag(-.3);   
        state = State.WAIT;   
        // waitStarted = System.currentTimeMillis();
        waitStarted = Timer.getFPGATimestamp(); 
      }
      else if (state == State.WAIT) {
        m_magazine.runLowerMag(0);
        m_magazine.runUpperMag(-.3); 
        if (Timer.getFPGATimestamp() - waitStarted > 0.5) {
          state = State.SHOOT2;
        }
      }
      else if(state == State.SHOOT2){
        m_magazine.runLowerMag(.2);
        m_magazine.runUpperMag(-.2);
      }

    }
    else{
      m_magazine.runLowerMag(0);
      m_magazine.runUpperMag(0);   
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
