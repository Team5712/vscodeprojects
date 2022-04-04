// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbArmDown extends CommandBase {

  Climber m_climber;
  private Timer glitchyLimitSwitchTimer = new Timer();
  private boolean timerRunning = false;
  /** Creates a new ClimbArmDown. */
  public ClimbArmDown(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    glitchyLimitSwitchTimer.reset();
    glitchyLimitSwitchTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.runClimber(.6);
    SmartDashboard.putNumber("Limit Switch Down Timer", glitchyLimitSwitchTimer.get());
    if(m_climber.getLimitSwitchLeftDown() && !timerRunning) {
      glitchyLimitSwitchTimer.reset();
      glitchyLimitSwitchTimer.start();
      timerRunning = true;
    }
    if(!m_climber.getLimitSwitchLeftDown()) {
      glitchyLimitSwitchTimer.reset();
      glitchyLimitSwitchTimer.stop();
      timerRunning = false;
    }
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
    if(glitchyLimitSwitchTimer.get() > 0.5) {
      return m_climber.getLimitSwitchLeftDown();
    }
    return false;
  }
}
