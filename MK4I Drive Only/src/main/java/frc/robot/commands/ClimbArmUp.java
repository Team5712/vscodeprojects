// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.sql.Time;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbArmUp extends CommandBase {
  private final Pigeon2 m_pigeon = new Pigeon2(13);
  private Timer timer = new Timer();
  private Timer glitchyLimitSwitchTimer = new Timer();
  private boolean timerRunning = false;
  private Climber m_climber;
  boolean armsUp = false;
  /** Creates a new Climb. */
  public ClimbArmUp(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    glitchyLimitSwitchTimer.reset();
    glitchyLimitSwitchTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //260000
    //274000
  //   if (m_climber.getLeftEncoderTicks() < -275000) {
  //     m_climber.runClimber(0);
  //   } 
  // else {
  //   m_climber.runClimber(-.5);
  // }
  if(timer.get()<0.25){m_climber.runClimber(-0.5);}
  else{m_climber.runClimber(-1);}
  

  SmartDashboard.putNumber("Limit Switch Up Timer", glitchyLimitSwitchTimer.get());

  if(m_climber.getLimitSwitchLeftUp() && !timerRunning) {
    glitchyLimitSwitchTimer.reset();
    glitchyLimitSwitchTimer.start();
    timerRunning = true;
  }
  if(!m_climber.getLimitSwitchLeftUp()) {
    glitchyLimitSwitchTimer.reset();
    glitchyLimitSwitchTimer.stop();
    timerRunning = false;
  }
  //System.out.println(m_climber.getLeftEncoderTicks());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timerRunning = false;
    glitchyLimitSwitchTimer.stop();
    m_climber.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return armsUp;
    if(glitchyLimitSwitchTimer.get() > .5) {
      return m_climber.getLimitSwitchLeftUp();
    }
    return false;

  }
}
