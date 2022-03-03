
#pragma once

#include <cmath>
#include <iostream>
#include <map>
#include <string>

#include "frc/WPILib.h"
#include <frc/IterativeRobot.h>
#include "AHRS.h"
#include "ctre/Phoenix.h"

#include <control_systems/GMDrive.h>
#include <control_systems/GyroCorrection.h>

#include "vision/Tracking.h"

using namespace std;

class Robot : public frc::IterativeRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledInit() override;

  void setSetpoints(float, float);
  void setAdjustments(float, float);

private:

  AHRS *gyro;

  float adjust_l = 0.0;
  float adjust_r = 0.0;

  float setpoint_l = 0.0;
  float setpoint_r = 0.0;

  frc::Joystick *joystick_l;
  frc::Joystick *joystick_r;
  frc::Joystick *joystick_aux;
  

  Tracking *vision_tracking;
  GyroCorrection *gyro_correction;

  const float ratio = (515 / (2 * 2 * M_PI));

  // TODO: implement this on the right side and fix the stuff
  // frc::DifferentialDrive *drive_system;
  GMDrive *drive;

  map<string, int> driver_buttons = {
	  {"trigger", 1},
	  {"left", 2},
	  {"right", 3},
	  {"middle", 4},
  };

  // input mapping for aux controller
  map<string, int> aux_buttons = {
	  {"a", 1},
  	  {"b", 2},
  	  {"x", 3},
  	  {"y", 4},
  	  {"left_bumper", 5},
  	  {"right_bumper", 6},
  	  {"back", 7},
  	  {"start", 8},
  	  {"left_click", 9},
  	  {"right_click", 10},
  };


  map<string, int> driver_axis = {
  	  {"trigger", 1},
  	  {"left", 2},
  	  {"right", 3},
  	  {"middle", 4},
    };

  map<string, int> aux_axis = {
	  {"trigger", 1},
	  {"left", 2},
	  {"right", 3},
	  {"middle", 4},
  };

  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Input methods for each respective joystick to update their respective systems
  //...............................................................................
  void handleDriverInput();
  void handleAuxiliaryInput();
  void resetDriveValues();
};
