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

public class Controller {
    private final XboxController m_controller = new XboxController(0);

    public Controller() {
        
    }

    public double getLeftY() {
        if(Math.abs(m_controller.getRawAxis(1)) < 0.05){
            return 0;
        }
        return m_controller.getRawAxis(1);
    }
    public double getLeftX() {
        if(Math.abs(m_controller.getRawAxis(0)) < 0.05){
            return 0;
        }
        return m_controller.getRawAxis(0);
    }
    public double getRightX() {
        if(Math.abs(m_controller.getRawAxis(4)) < 0.05){
            return 0;
        }
        return m_controller.getRawAxis(4);
    }

}
