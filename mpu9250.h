#ifndef MPU9250_H_
#define MPU9250_H_

#include "mpu.h"

void MPU9250_calibrate(float *acceleration_bias, float *degree_bias);
void MPU9250_Accelerometer(float *acceleration);
void MPU9250_Gyroscope(float *degrees);
void MPU9250_Temperature(float *temperature);
void MPU9250_Motion(float *acceleration, float *degrees);

#endif /* MPU9250_H_ */
