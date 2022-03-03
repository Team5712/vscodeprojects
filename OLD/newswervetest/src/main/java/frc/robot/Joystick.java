/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */

public class Joystick {
    private final XboxController m_controller = new XboxController(0);

    public Joystick() {
    }

    public double getLeftY() {
        return m_controller.getRawAxis(1);
    }
    public double getLeftX() {
        return m_controller.getRawAxis(0);
    }
    public double getRightX() {
        return m_controller.getRawAxis(4);
    }

}
