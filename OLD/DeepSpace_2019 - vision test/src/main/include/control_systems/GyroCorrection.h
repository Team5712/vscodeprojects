#ifndef SRC_CONTROL_SYSTEMS_GYROCORRECTION_H_
#define SRC_CONTROL_SYSTEMS_GYROCORRECTION_H_

#include <vector>
#include <iostream>

#include "AHRS.h"

using namespace std;

class GyroCorrection {
private:

    AHRS *gyro;
  
public:

    GyroCorrection(AHRS *gyro);
    vector<float> adjustDrive();

	float gyro_error = 0;
	float gyro_correction = 0;

	float l_gyro_adjust;
	float r_gyro_adjust;

    float gyro_min = 4.0;
    float gyro_Kp = 9.0;
    int gyro_skip = 2;

};

#endif /* SRC_CONTROL_SYSTEMS_GYROCORRECTION_H_ */