// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class RunLowerMag extends CommandBase {
  /** Creates a new RunLowerMag. */

  private Magazine m_magazine;

  public RunLowerMag(Magazine magazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_magazine = magazine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_magazine.runLowerMag(.5);
    //System.out.println("UPPER BALL SENSOR" + m_magazine.getLowerBallSensor());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_magazine.runLowerMag(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
