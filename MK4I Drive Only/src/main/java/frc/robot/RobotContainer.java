// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Change sensor in can coder to boot to absolute
//Redeploy robot code
//Double check that pheniox tuner is still boot to absolute (that will be after enabled)
//If they are boot to absolute is weird change values
//Back out changes
//2 after democat
///Try their version make sure to remove ssds code from library teams are using there folk.
//Feel free to reach out.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.auto.follower;
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
  private final XboxController m_testcontroller = new XboxController(2);
  SendableChooser<String> m_chooser = new SendableChooser<>();
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
    m_chooser.setDefaultOption("3 Ball Right", "3 Ball Right");
    m_chooser.addOption("5 Ball Right", "5 Ball Right");
    m_chooser.addOption("Nothing", "Nothing");
    m_chooser.addOption("2 Ball Left", "2 Ball Left");
    m_chooser.addOption("Straight", "Straight");
    SmartDashboard.putData(m_chooser);

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller1.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller1.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(-m_controller1.getRightX() + m_limelight.turnToTarget()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

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
      //m_controller2.setRumble(RumbleType.kLeftRumble, 1);
      //m_controller1.setRumble(RumbleType.kLeftRumble, 1);
    // Back button zeros the gyroscope
    new Button(m_controller1::getAButton)
           // No requirements because we don't need to interrupt anything
           .whenPressed(m_drivetrainSubsystem::zeroGyroscope);


            // new Button(m_controller::getRightBumper)
            // No requirements because we don't need to interrupt anything
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

  new Button(m_controller2::getAButton)
    .whenHeld(new CalibrateHood(m_shooter));

  new Button(m_controller2::getRightBumper)
  // No requirements because we don't need to interrupt anything
    .whenHeld(new ShootCustom(m_shooter, m_magazine,11500,-3));
    //Box shot -4 for CustomHood 11500 for ShootCustom
    //Safe zone short shot -8 13000
    //Safe zone long shot -7.8 14000
    //Auto deep shot -8.2, 16000

    new Button(m_controller2::getXButton)
    .whenHeld(new PickUpBall(m_intake, m_magazine));
  // No requirements because we don't need to interrupt anything.w
    // .toggleWhenPressed(new CustomHoodAngle(m_shooter, -8.2));

  new Button(m_controller2::getYButton)
    // No requirements because we don't need to interrupt anything
    .whenHeld(new ShootCustom(m_shooter, m_magazine,6000,-5));
    // .whenHeld(new CustomHoodAngle(m_shooter, -5));

  new Button(m_controller2::getBButton)
      // No requirements because we don't need to interrupt anything
        .whenHeld(new ShootCustom(m_shooter, m_magazine,13000,-6));

  new Button(m_controller2::getLeftBumper)
    // No requirements because we don't need to interrupt anything
      .whenHeld(new RunLowerMag(m_magazine)
      .alongWith(new RunUpperMag(m_magazine)));

  new Button(m_controller1::getRightBumper)  
              .whenHeld(new PickUpBall(m_intake, m_magazine)
              .alongWith(new IntakeDown(m_intake)));           
  // No requirements because we don't need to interrupt anything

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
  //              .andThen(new ClimbArmDown(m_climber)))))))))))))); // arms are retracted m_testcontroller
    
  // new Button(m_controller1::getAButton)
  //           .whenPressed(new ClimbArmUp(m_climber));
  
  // new Button(m_controller1::getBButton)
  //           .whenPressed(new ClimbArmDown(m_climber));
  

    new Button(m_testcontroller::getAButton)
  //              // No requirements because we don't need to interrupt anything
                .whenHeld(new ClimbArmDown(m_climber));
    new Button(m_testcontroller::getYButton)
  //              // No requirements because we don't need to interrupt anything
                .whenHeld(new ClimbArmUp(m_climber)
                .alongWith(new ShootStop(m_shooter)));
    new Button(m_testcontroller::getXButton)
  //              // No requirements because we don't need to interrupt anything
                .whenPressed(new ClimbArmForward(m_climber));
    new Button(m_testcontroller::getBButton)
  //              // No requirements because we don't need to interrupt anything
                .whenPressed(new ClimbArmBack(m_climber));      

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autonChosen = m_chooser.getSelected();
    m_shooter.zeroEncoderOfHood();
    m_drivetrainSubsystem.zeroGyroscope();
// 1. Create trajectory settings
TrajectoryConfig trajectoryConfig = Constants.auto.follower.T_CONFIG;
TrajectoryConfig trajectoryConfig3rdPath = Constants.auto.follower.T_CONFIG_3rdPath;

Trajectory trajectoryStraight = TrajectoryGenerator.generateTrajectory(
  new Pose2d(-.7, 0, new Rotation2d(-10)),
  List.of(
          new Translation2d(-.2, .5),
          new Translation2d(.1, .8)),
  new Pose2d(.2, 1.7, Rotation2d.fromDegrees(-55)),
  trajectoryConfig);

Trajectory trajectoryLeft1 = TrajectoryGenerator.generateTrajectory(
  new Pose2d(0, 0, new Rotation2d(0)),
  List.of(
          new Translation2d(-.5, 0),
          new Translation2d(-.75, 0)),
  new Pose2d(-1, 0, Rotation2d.fromDegrees(10)),
  trajectoryConfig);

  Trajectory trajectoryLeft2 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(-1, 0, new Rotation2d(10)),
    List.of(
            new Translation2d(-.3, .75),
            new Translation2d(.2, 1.5)),
    new Pose2d(0.35, 1.75, Rotation2d.fromDegrees(-70)), //.4 2.15
    trajectoryConfig);

// 2. Generate trajectory
Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
  new Pose2d(0, 0, new Rotation2d(0)),
  List.of(
          new Translation2d(-.3, 0),
          new Translation2d(-.5, 0)),
  new Pose2d(-.7, 0, Rotation2d.fromDegrees(-10)),
  trajectoryConfig);

  Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
  new Pose2d(-.7, 0, new Rotation2d(-10)),
  List.of(
          new Translation2d(-.2, .5),
          new Translation2d(.1, .8)),
  new Pose2d(.2, 1.7, Rotation2d.fromDegrees(-55)), //.4 2.15 .35 1.75
  trajectoryConfig);

  Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
  new Pose2d(0, 0, new Rotation2d(-65)), //.35 1.75
  List.of(
          new Translation2d(-.9 * .3, 2.5 * .3),
          new Translation2d(-.9 * .6, 2.5 * .6)),
          //Closer to tower decrease 5.5
          //Closer to middle of field increase -.5
  //5.6 worked then ran into wall I suggest lowering it by 1 and tuning from there
  new Pose2d(-0.9, 2.5, Rotation2d.fromDegrees(-65)),
  trajectoryConfig3rdPath);


// 3. Define PID controllers for tracking trajectory
PIDController xController = Constants.auto.follower.X_PID_CONTROLLER;
PIDController yController = Constants.auto.follower.Y_PID_CONTROLLER;
ProfiledPIDController thetaController = Constants.auto.follower.ROT_PID_CONTROLLER;
thetaController.enableContinuousInput(-Math.PI, Math.PI);

SwerveControllerCommand swerveControllerCommandStraight = new SwerveControllerCommand(
  trajectoryStraight,
  m_drivetrainSubsystem::getPose2d,
  m_drivetrainSubsystem.getKinematics(),
  xController,
  yController,
  thetaController,
  m_drivetrainSubsystem::setAllStates,
  m_drivetrainSubsystem);

SwerveControllerCommand swerveControllerCommandLeft1 = new SwerveControllerCommand(
  trajectoryLeft1,
  m_drivetrainSubsystem::getPose2d,
  m_drivetrainSubsystem.getKinematics(),
  xController,
  yController,
  thetaController,
  m_drivetrainSubsystem::setAllStates,
  m_drivetrainSubsystem);

  SwerveControllerCommand swerveControllerCommandLeft2 = new SwerveControllerCommand(
  trajectoryLeft2,
  m_drivetrainSubsystem::getPose2d,
  m_drivetrainSubsystem.getKinematics(),
  xController,
  yController,
  thetaController,
  m_drivetrainSubsystem::setAllStates,
  m_drivetrainSubsystem);
// 4. Construct command to follow trajectory
SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
  trajectory1,
  m_drivetrainSubsystem::getPose2d,
  m_drivetrainSubsystem.getKinematics(),
  xController,
  yController,
  thetaController,
  m_drivetrainSubsystem::setAllStates,
  m_drivetrainSubsystem);

SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
  trajectory2,
  m_drivetrainSubsystem::getPose2d,
  m_drivetrainSubsystem.getKinematics(),
  xController,
  yController,
  thetaController,
  m_drivetrainSubsystem::setAllStates,
  m_drivetrainSubsystem);

  SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
    trajectory3,
    m_drivetrainSubsystem::getPose2d,
    m_drivetrainSubsystem.getKinematics(),
    xController,
    yController,
    thetaController,
    m_drivetrainSubsystem::setAllStates,
    m_drivetrainSubsystem); 

  // return new SequentialCommandGroup(
  //   new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
  //   swerveControllerCommand1,
  //   new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory2.getInitialPose())),
  //   swerveControllerCommand2,
  //   new InstantCommand(() -> m_drivetrainSubsystem.stopModules())
  //   );

  
  // 3 Ball Right
  if (autonChosen == "3 Ball Right") {
    m_shooter.moveHood(0);
    m_shooter.stop();
    m_magazine.runUpperMag(0);
    return new AutoPickUpBall(m_intake, m_magazine, m_shooter,11500,-4).alongWith(
    new SequentialCommandGroup(
    new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
    swerveControllerCommand1,
    new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
    new AutoShootCommand(m_magazine, m_shooter, 11500),
    new WaitCommand(2),
    new AutoShootCommand(m_magazine, m_shooter, 11500),
    new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory2.getInitialPose())),
    swerveControllerCommand2,
    new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
    new WaitCommand(1),
    new AutoShootCommand(m_magazine, m_shooter, 11500)));
  }
  else if (autonChosen == "5 Ball Right") {
    m_shooter.moveHood(0);
    m_shooter.stop();
    m_magazine.runUpperMag(0);
    return new ParallelRaceGroup(
      new AutoPickUpBall(m_intake, m_magazine, m_shooter,11500,-4),
      new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
        swerveControllerCommand1,
        new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
        new AutoShootCommand(m_magazine, m_shooter, 11500),
        new WaitCommand(1),
        new AutoShootCommand(m_magazine, m_shooter, 11500),
        new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory2.getInitialPose())),
        swerveControllerCommand2,
        new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
        new WaitCommand(1),
        new AutoShootCommand(m_magazine, m_shooter, 11500))).andThen(
        new AutoPickUpBall(m_intake, m_magazine, m_shooter,16000,-8.2).alongWith(new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory3.getInitialPose())),
        swerveControllerCommand3, 
        new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
        new WaitCommand(1),
        new AutoShootCommand(m_magazine, m_shooter, 16000),
        new WaitCommand(1),
        new AutoShootCommand(m_magazine, m_shooter, 16000) 
      )));
  }
      // 2 Ball Left
else if (autonChosen == "2 Ball Left") {
    m_shooter.moveHood(0);
    m_shooter.stop();
    m_magazine.runUpperMag(0);
    return new AutoPickUpBall(m_intake, m_magazine, m_shooter,11500,-4).alongWith(
      new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectoryLeft1.getInitialPose())),
      swerveControllerCommandLeft1,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules())),
      new AutoShootCommand(m_magazine, m_shooter, 11500),
      new WaitCommand(2),
      new AutoShootCommand(m_magazine, m_shooter, 11500),
      new WaitCommand(2),
      new AutoShootCommand(m_magazine, m_shooter, 11500));
  }
  else if (autonChosen == "Straight") {
    System.out.println("STRAIGHT");
   return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectoryStraight.getInitialPose())),
      swerveControllerCommandStraight);
  }
else {
  return null;
}
  
  

//5 ball

}
  

  // m_shooter.moveHood(0);
  // m_shooter.stop();
  // m_magazine.runUpperMag(0);
  // return new SequentialCommandGroup(
  //   new ParallelRaceGroup(
  //     new AutoPickUpBall(m_intake, m_magazine, m_shooter,11500,-4),
  //     new SequentialCommandGroup(
  //       new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
  //       swerveControllerCommand1,
  //       new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
  //       new AutoShootCommand(m_magazine, m_shooter, 11500),
  //       new WaitCommand(1),
  //       new AutoShootCommand(m_magazine, m_shooter, 11500),
  //       new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory2.getInitialPose())),
  //       swerveControllerCommand2,
  //       new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
  //       new WaitCommand(1),
  //       new AutoShootCommand(m_magazine, m_shooter, 11500)
  //     )
  //   ),
  //   new ParallelCommandGroup(
  //     new AutoPickUpBall(m_intake, m_magazine, m_shooter,16000,-8.2),
  //     new SequentialCommandGroup(
  //       new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory3.getInitialPose())),
  //       swerveControllerCommand3, 
  //       new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
  //       new WaitCommand(1),
  //       new AutoShootCommand(m_magazine, m_shooter, 16000),
  //       new WaitCommand(1),
  //       new AutoShootCommand(m_magazine, m_shooter, 16000) 
  //     )
  //   )
  // );
    

  // return new AutoPickUpBall(m_intake, m_magazine);

// 5. Add some init and wrap-up, and return everything


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
