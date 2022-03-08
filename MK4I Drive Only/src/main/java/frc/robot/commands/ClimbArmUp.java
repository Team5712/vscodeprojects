// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbArmUp extends CommandBase {
  private final Pigeon2 m_pigeon = new Pigeon2(13);
  
  private Climber m_climber;
  boolean armsUp = false;
  /** Creates a new Climb. */
  public ClimbArmUp(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climber.getLimitSwitchRightUp() && m_climber.getLimitSwitchLeftUp()){
      if (m_pigeon.getPitch() > -.04 ){
        
        m_climber.runClimber(-.25);
      }
     else{
       m_climber.runClimber(0);
     }
    }
    else{
      m_climber.runClimber(0);
      armsUp = true;
    }
 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armsUp;
  }
}
