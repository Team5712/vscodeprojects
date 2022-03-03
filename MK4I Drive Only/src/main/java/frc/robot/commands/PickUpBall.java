// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class PickUpBall extends CommandBase {
  /** Creates a new PickUpBall. */

  private Intake m_intake;
  private Magazine m_magazine;

  public PickUpBall(Intake intake, Magazine magazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_magazine = magazine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // m_intake.moveIntake(-.4);
    // m_magazine.runLowerMag(0.7);
    // m_magazine.runUpperMag(-.7);

    //System.out.println("BALL SENSOR VALUE:     " + Constants.UPPER_BALL_SENSOR_THRESHOLD);
    if (m_magazine.getUpperBallSensor() < Constants.UPPER_BALL_SENSOR_THRESHOLD) {
      m_intake.moveIntake(-.4);
      m_magazine.runLowerMag(0.5);
      m_magazine.runUpperMag(-.5);
    } 
    if (m_magazine.getUpperBallSensor() > Constants.UPPER_BALL_SENSOR_THRESHOLD) {
      m_intake.moveIntake(-.4);
      m_magazine.runLowerMag(0.5);
      m_magazine.runUpperMag(0);
    } 
    if (m_magazine.getUpperBallSensor() > Constants.UPPER_BALL_SENSOR_THRESHOLD && m_magazine.getLowerBallSensor() > Constants.LOWER_BALL_SENSOR_THRESHOLD) {
      m_intake.moveIntake(-.4);
      m_magazine.runLowerMag(0.5);
      m_magazine.runUpperMag(0);
    } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.moveIntake(0);
    m_magazine.runLowerMag(0);
    m_magazine.runUpperMag(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
