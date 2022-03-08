// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ReverseIntake extends CommandBase {
  /** Creates a new ReverseIntake. */

  private Intake m_intake;
  private Magazine m_magazine;
  private Shooter m_shooter;

  public ReverseIntake(Intake intake, Magazine magazine, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_magazine = magazine;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.moveIntake(1);
    m_magazine.runLowerMag(-1);
    m_magazine.runUpperMag(.3);
    m_shooter.reverseShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.moveIntake(0.0);
    m_magazine.runLowerMag(0);
    m_magazine.runUpperMag(0);
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
