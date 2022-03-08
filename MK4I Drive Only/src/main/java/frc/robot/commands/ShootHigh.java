// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShootHigh extends CommandBase {
  /** Creates a new ShootHigh. */

  private Shooter m_shooter;
  private Magazine m_magazine;

  public ShootHigh(Shooter shooter, Magazine magazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_magazine = magazine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.shootHigh();
    // System.out.println(m_shooter.getShooterError());
    // if (m_shooter.getShooterError() < 100) {
    //   System.out.println(m_shooter.getShooterError());
    //   m_magazine.runLowerMag(.3);
    //   m_magazine.runUpperMag(-.3);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
