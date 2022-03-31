// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class TurnToHub extends CommandBase {

  DrivetrainSubsystem m_drivetrain;
  Limelight m_limelight;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private Translation2d hub_position = new Translation2d(8.22, 4.11);

  /** Creates a new TurnToHub. */
  public TurnToHub(DrivetrainSubsystem drivetrain, Limelight limelight, DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robot_pose = m_drivetrain.getPose2d();
    Translation2d robot_position = robot_pose.getTranslation();

    double angle = Math.atan2(robot_position.getY() - hub_position.getY(), robot_position.getX() - hub_position.getX());
    double angle_error = angle - m_drivetrain.getGyroscopeRotation().getRadians();

    m_drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            Math.max(-1, Math.min(angle_error, 1)) * Constants.swerve.MAX_ANG_ACCEL,
            m_drivetrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
