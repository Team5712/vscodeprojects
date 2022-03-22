// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbArmDown extends CommandBase {
  private Climber m_climber;
  boolean armsDown = false;
  /** Creates a new Climb. */
  public ClimbArmDown(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  //true limit switches means they are not pressed
  @Override
  public void execute() {
  //   if (m_climber.getLeftEncoderTicks() > -5000) {
  //     m_climber.runClimber(0);
  //   } 
  // else {
  //   m_climber.runClimber(.5);
  // }
  m_climber.runClimber(.6);
  // m_climber.getLeftEncoderTicks();
  //System.out.println(m_climber.getLeftEncoderTicks());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return armsDown;
    return false;
  }
}
