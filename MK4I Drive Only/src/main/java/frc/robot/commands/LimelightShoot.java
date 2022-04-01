// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class LimelightShoot extends CommandBase {
  private Shooter m_shooter;
  private Magazine m_magazine;
  private Limelight m_limelight;

  private final Timer timer = new Timer();

  /** Creates a new LimelightShoot. */
  public LimelightShoot(Shooter shooter, Magazine magazine, Limelight limelight) {
    m_shooter = shooter;
    m_magazine = magazine;
    m_limelight = limelight;
    addRequirements(shooter);
  }

  // double[] nums = {7.5, 0.9, -4.5, -6, -7.4, -9, -13};

  private double calculateRPM() {
    double ty = m_limelight.getY();
    // return -195.245 * ty + 12707;
    if (ty < 7.5 && ty >= 0.9) {
      return -196.97 * ty + 12977;
    }
    if (ty < 0.9 && ty >= -4.5) {
      return -55.5556 * ty + 12850;
    }
    if (ty < -4.5 && ty >= -6) {
      return -266.667 * ty + 11900;
    }
    if (ty < -6 && ty >= -7.4) {
      return -214.286 * ty + 12214;
    }
    if (ty < -7.4 && ty >= -9) {
      return -375 * ty + 11025;
    }
    if (ty < -9 && ty >= -13) {
      return -400 * ty + 10800;
    }
    return 0;
  }

  private double calculateHoodAngle() {
    double ty = m_limelight.getY();
    if (ty < 7.5 && ty >= 0.9) {
      return Math.min(Math.max(0.30303 * ty - 5.27273, -10), 0);
    }
    if (ty < 0.9 && ty >= -4.5) {
      return 0.518519 * ty - 5.46667;
    }
    if (ty < -4.5 && ty >= -6) {
      return Math.min(Math.max(.466667 * ty - 5.7, -10), 0);
    }
    if (ty < -6 && ty >= -7.4) {
      return Math.min(Math.max(.0714286 * ty - 8.07, -10), 0);
    }
    if (ty < -7.4 && ty >= -9) {
      return Math.min(Math.max(.25 * ty - 6.75, -10), 0);
    }
    if (ty < -9 && ty >= -13) {
      return Math.min(Math.max(0.05 * ty - 8.55, -10), 0);
    }
    return 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_RPM = calculateRPM();
    double m_setHoodAngle = calculateHoodAngle();
    m_shooter.customShootHigh(m_RPM);
    double currentPosition = m_shooter.getHoodPosition();
    double errorDis = currentPosition - m_setHoodAngle;
    m_shooter.moveHood(errorDis * -.03);
    if (timer.get() > .25) {
      if (m_shooter.leftSpeed() < m_RPM * 1.1 && m_shooter.leftSpeed() > m_RPM * 0.9) {
        if (m_magazine.getUpperBallSensor() < Constants.UPPER_BALL_SENSOR_THRESHOLD) {
          m_magazine.runLowerMag(.2);
          m_magazine.runUpperMag(-.2);
        } else {
          m_magazine.runLowerMag(0);
          m_magazine.runUpperMag(-.2);
        }

      } else {
        m_magazine.runLowerMag(0);
        m_magazine.runUpperMag(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.customShootHigh(6000);
    m_magazine.runLowerMag(0);
    m_magazine.runUpperMag(0);
    m_shooter.moveHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
