// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;
import frc.robot.commands.Auto.RunBasicTrajectory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Magazine m_magazine = new Magazine();
  private final Climber m_climber = new Climber();
  private final Limelight m_limelight = new Limelight();
  private final XboxController m_controller1 = new XboxController(0);
  private final XboxController m_controller2 = new XboxController(1);
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  private double visionCorrection;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller1.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller1.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(-m_controller1.getRightX() + m_limelight.turnToTarget()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    m_climber.runToPosition(5);
    m_compressor.enableAnalog(115, 120);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller1::getAButton)
           // No requirements because we don't need to interrupt anything
           .whenPressed(m_drivetrainSubsystem::zeroGyroscope);


            // new Button(m_controller::getRightBumper)
            // // No requirements because we don't need to interrupt anything
            // .whenHeld(new ShootHigh(m_shooter)
            // .alongWith(new RunIntake(m_intake)
            // .alongWith(new RunLowerMag(m_magazine)
            // .alongWith(new RunUpperMag(m_magazine)))));

  // new Button(m_controller::getXButton)
  //             // No requirements because we don't need to interrupt anything
  //             .whenHeld(new HoodForward(m_shooter));

  // new Button(m_controller::getYButton)
  //             // No requirements because we don't need to interrupt anything
  //             .whenHeld(new HoodBackward(m_shooter));

  // new Button(m_controller::getXButton)
  //             // No requirements because we don't need to interrupt anything
  //             .whenHeld(new RunIntake(m_intake));

  // new Button(m_controller::getYButton)
  //             // No requirements because we don't need to interrupt anything
  //             .whenHeld(new ReverseIntake(m_intake));

  // new Button(m_controller::getAButton)
  //             // No requirements because we don't need to interrupt anything
  //             .whenHeld(new RunLowerMag(m_magazine));

  //new Button(m_controller1::getStartButton)
  //.whenHeld(new IntakeDown(m_intake));

  new Button(m_controller1::getXButton)
    .whenHeld(new CalibrateHood(m_shooter));

  new Button(m_controller2::getRightBumper)
  // No requirements because we don't need to interrupt anything
    .whenHeld(new Shoot(m_shooter, m_magazine));
  new Button(m_controller2::getLeftBumper)
    // No requirements because we don't need to interrupt anything
      .whenHeld(new RunLowerMag(m_magazine)
      .alongWith(new RunUpperMag(m_magazine)));

  new Button(m_controller1::getRightBumper)
              // No requirements because we don't need to interrupt anything
              .whenHeld(new PickUpBall(m_intake, m_magazine)
              .alongWith(new IntakeDown(m_intake)));

  new Button(m_controller1::getLeftBumper)
              // No requirements because we don't need to interrupt anything
              .whenHeld(new ReverseIntake(m_intake, m_magazine, m_shooter));
  
  new Button(m_controller1::getYButton)
              // No requirements because we don't need to interrupt anything
              .whenHeld(new TurnToTarget(m_limelight));

  // new Button(m_controller1::getStartButton)
  //              // No requirements because we don't need to interrupt anything
  //              .whenPressed(new ClimbArmDown(m_climber)
  //              .andThen(new WaitCommand(1))
  //              .andThen(new ClimbArmUp6in(m_climber) //only go up 6 inches
  //              .andThen(new ClimbArmForward(m_climber) //pneumatics out
  //              .andThen(new WaitCommand(1)) //wait 1 second
  //              .andThen(new ClimbArmUp(m_climber) // arms finish extending
  //              .andThen(new ClimbArmBack(m_climber) // pneumatics in
  //              .andThen(new WaitCommand(1) //wait 1 second
  //              .andThen(new ClimbArmDown(m_climber) // arms are retracted
  //              .andThen(new WaitCommand(3))
  //              .andThen(new ClimbArmUp6in(m_climber) //only go up 6 inches
  //              .andThen(new ClimbArmForward(m_climber) //pneumatics out
  //              .andThen(new WaitCommand(1) //wait 1 second
  //              .andThen(new ClimbArmUp(m_climber) // arms finish extending
  //              .andThen(new ClimbArmBack(m_climber) // pneumatics in
  //              .andThen(new WaitCommand(1)) //wait 1 second
  //              .andThen(new ClimbArmDown(m_climber)))))))))))))); // arms are retracted
    
  // new Button(m_controller1::getAButton)
  //           .whenPressed(new ClimbArmUp(m_climber));
  
  // new Button(m_controller1::getBButton)
  //           .whenPressed(new ClimbArmDown(m_climber));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    RunBasicTrajectory testauto = new RunBasicTrajectory(m_drivetrainSubsystem, "New Path"); //"New New New New New New New Path");
    return testauto;
  }
  // Testing
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
