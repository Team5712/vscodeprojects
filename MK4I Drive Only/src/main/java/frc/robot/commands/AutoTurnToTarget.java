// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AutoTurnToTarget extends DefaultDriveCommand {
  /** Creates a new PickUpBall. */

  private Limelight rotateRobot = new Limelight();
  private boolean turningDone;
  private double errorTurn;
  private long safteyTimer;
  private double timeToTurn = 2;
  DrivetrainSubsystem m_drivetrainSubsystem;
  Joystick autoJoystick = new Joystick(4);
  static private double output = 0;

  public AutoTurnToTarget(DrivetrainSubsystem drivetrainSubsystem) {
    super(drivetrainSubsystem, ()->0,()->0,()->output);
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turningDone = false;
    safteyTimer=System.currentTimeMillis();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //super();
    errorTurn = rotateRobot.getX();
    System.out.println("Error in x direction: "+errorTurn);
    output = rotateRobot.turnToTarget();
    // if(Math.abs(errorTurn)<0.05||(System.currentTimeMillis()-safteyTimer>5000)){
    //     turningDone = true;
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return turningDone;
  }
}
