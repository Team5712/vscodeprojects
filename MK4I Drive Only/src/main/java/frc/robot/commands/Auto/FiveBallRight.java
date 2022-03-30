// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoPickUpBall;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallRight extends SequentialCommandGroup {
  private Shooter m_shooter;
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Intake m_intake;
  private Magazine m_magazine;

  /** Creates a new FiveBallRight. */
  public FiveBallRight(Shooter shooter, DrivetrainSubsystem drivetrainSubsystem, Intake intake, Magazine magazine) {
    m_shooter = shooter;
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_intake = intake;
    m_magazine = magazine;

    m_shooter.zeroEncoderOfHood();
    m_drivetrainSubsystem.zeroGyroscope();

    TrajectoryConfig trajectoryConfig = Constants.auto.follower.T_CONFIG;

    Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(.6, 1.7, new Rotation2d(-60)), // .35 1.75
        List.of(
            new Translation2d(.4, 2),
            new Translation2d(.2 , 2.5)),
        // Closer to tower decrease 5.5
        // Closer to middle of field increase -.5
        // 5.6 worked then ran into wall I suggest lowering it by 1 and tuning from
        // there
        new Pose2d(0, 3, Rotation2d.fromDegrees(-65)),
        trajectoryConfig);

    PIDController xController = Constants.auto.follower.X_PID_CONTROLLER;
    PIDController yController = Constants.auto.follower.Y_PID_CONTROLLER;
    ProfiledPIDController thetaController = Constants.auto.follower.ROT_PID_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

  SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
      trajectory3,
      m_drivetrainSubsystem::getPose2d,
      m_drivetrainSubsystem.getKinematics(),
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::setAllStates,
      m_drivetrainSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
          new ThreeBallRight(m_shooter, m_drivetrainSubsystem, m_intake, m_magazine).andThen(
                  new AutoPickUpBall(m_intake, m_magazine, m_shooter, 16000, -8.2).raceWith(new SequentialCommandGroup(
                      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory3.getInitialPose())),
                      swerveControllerCommand3,
                      new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
                      new WaitCommand(1),
                      new AutoShootCommand(m_magazine, m_shooter, 16000),
                      new WaitCommand(1),
                      new AutoShootCommand(m_magazine, m_shooter, 16000))))
    );
  }
}
