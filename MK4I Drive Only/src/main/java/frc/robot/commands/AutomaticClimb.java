// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;

public class AutomaticClimb extends SequentialCommandGroup {
  public AutomaticClimb(Climber climber) {
    addCommands(
        new SequentialCommandGroup(
            new ClimbArmDown(climber),
            new WaitCommand(0.5),
            new SequentialCommandGroup(
                new ClimbArmUp6in(climber),
                new WaitCommand(0.5),
                new ClimbArmsOut(climber),
                // new WaitCommand(0.5),
                new ClimbArmUp(climber)),
            new SequentialCommandGroup(
                new ClimbArmsIn(climber),
                new WaitCommand(1),
                new ClimbArmDown(climber))));
  }
}
