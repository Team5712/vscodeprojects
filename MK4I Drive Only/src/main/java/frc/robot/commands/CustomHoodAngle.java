// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class CustomHoodAngle extends CommandBase {
  /** Creates a new HoodBackward. */

  private Shooter m_shooter;
  private double m_setHoodAngle;
  private boolean commandFinished;

  public CustomHoodAngle(Shooter shooter, double setHoodAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_setHoodAngle = setHoodAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //0 is farthest back forward -10 is farthest forward.
    System.out.println("TOGGLED ON");
    double currentPosition = m_shooter.getHoodPosition();
    double errorDis = currentPosition-m_setHoodAngle;
    // System.out.println("Location is : "+m_shooter.getHoodPosition());
    m_shooter.moveHood(errorDis*-.03);

    ///System.out.println("Error to targert: "+errorDis);
    // System.out.println("Power provided to targert: "+errorDis*.01);
    //m_shooter.moveHoos(-.05);

    // m_shooter.moveHood(errorDis*.0001);
    
    /*
    if(Math.abs(errorDis)<1){
        commandFinished = true;
    }
    else{
        m_shooter.moveHood(errorDis*.05);
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.moveHood(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
