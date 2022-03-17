// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutoIntakeDown extends CommandBase {

    private Intake m_intake;
    private boolean commandFinished;
    private long endTimer = 0;
    double runTimeCommand = 0;

  /** Creates a new IntakeDown. */
  public AutoIntakeDown(Intake intake, double runTimeCommandSec) {
    m_intake = intake;

    runTimeCommand = runTimeCommandSec;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandFinished = false;
    endTimer = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis()-endTimer<(runTimeCommand*1000)){
    //System.out.println("BUTTON PRESSSEED");
      m_intake.moveSolenoid(true);
    }
    else{
      end(true);
      commandFinished = true;
      isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.moveSolenoid(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
