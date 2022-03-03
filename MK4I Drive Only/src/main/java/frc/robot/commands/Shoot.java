// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */

  private Shooter m_shooter;
  private Magazine m_magazine;
  private boolean upperBallGone = false;

  public Shoot(Shooter shooter, Magazine magazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //start shooter
    m_shooter.shootHigh();
    //check if error is less than 100 RPM
    if (m_shooter.getShooterError() < 100) {
      //run upper mag and set boolean to true
      m_magazine.runUpperMag(-.5);
      upperBallGone = true;
      //if upper ball is gone, run lower mag
      if (upperBallGone) {
        m_magazine.runLowerMag(.5);
        //if upper ball is gone, and lower mag threshold 
        if (upperBallGone && m_magazine.getLowerBallSensor() < Constants.LOWER_BALL_SENSOR_THRESHOLD)
          m_shooter.stop();
          m_magazine.stop();
      }
    }


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
