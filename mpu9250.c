#include "mpu9250.h"

void MPU9250_calibrate(float *acceleration_bias, float *degree_bias){
    uint8_t raw_accel[6];
    uint8_t raw_gyro[6];

    int16_t accel[3];
    int16_t deg[3];
    int16_t accel_bias[3];
    int16_t deg_bias[3];
    int64_t accel_sum[3] = {0, 0, 0};
    int64_t deg_sum[3] = {0, 0, 0};
    uint16_t packets = 512;

    uint16_t i;

    for(i = 0; i < packets; i++){
        MPU_ReadAccelerometer(raw_accel);
        MPU_ReadGyroscope(raw_gyro);

        accel[0] = (raw_accel[0] << 8) | raw_accel[1];
        accel[1] = (raw_accel[2] << 8) | raw_accel[3];
        accel[2] = (raw_accel[4] << 8) | raw_accel[5];

        deg[0] = (raw_gyro[0] << 8) | raw_gyro[1];
        deg[1] = (raw_gyro[2] << 8) | raw_gyro[3];
        deg[2] = (raw_gyro[4] << 8) | raw_gyro[5];

        accel_sum[0] += accel[0];
        accel_sum[1] += accel[1];
        accel_sum[2] += accel[2];

        deg_sum[0] += deg[0];
        deg_sum[1] += deg[1];
        deg_sum[2] += deg[2];
    }

    accel_bias[0] = accel_sum[0] / packets;
    accel_bias[1] = accel_sum[1] / packets;
    accel_bias[2] = accel_sum[2] / packets;

    deg_bias[0] = deg_sum[0] / packets;
    deg_bias[1] = deg_sum[1] / packets;
    deg_bias[2] = deg_sum[2] / packets;

    acceleration_bias[0] = (float)(2 * accel_bias[0])/32768;
    acceleration_bias[1] = (float)(2 * accel_bias[1])/32768;
    acceleration_bias[2] = (float)(2 * accel_bias[2])/32768;

    degree_bias[0] = (float)(250 * deg_bias[0])/32768;
    degree_bias[1] = (float)(250 * deg_bias[1])/32768;
    degree_bias[2] = (float)(250 * deg_bias[2])/32768;
}

void MPU9250_Accelerometer(float *acceleration){
    uint8_t raw_accel[6];
    int16_t accel[3];

    MPU_ReadAccelerometer(raw_accel);

    accel[0] = (raw_accel[0] << 8) | raw_accel[1];
    accel[1] = (raw_accel[2] << 8) | raw_accel[3];
    accel[2] = (raw_accel[4] << 8) | raw_accel[5];

    acceleration[0] = (float)(2 * accel[0])/32768;
    acceleration[1] = (float)(2 * accel[1])/32768;
    acceleration[2] = (float)(2 * accel[2])/32768;
}

void MPU9250_Gyroscope(float *degrees){
    uint8_t raw_gyro[6];
    int16_t deg[3];

    MPU_ReadGyroscope(raw_gyro);

    deg[0] = (raw_gyro[0] << 8) | raw_gyro[1];
    deg[1] = (raw_gyro[2] << 8) | raw_gyro[3];
    deg[2] = (raw_gyro[4] << 8) | raw_gyro[5];

    degrees[0] = (float)(250 * deg[0])/32768.0;
    degrees[1] = (float)(250 * deg[1])/32768.0;
    degrees[2] = (float)(250 * deg[2])/32768.0;
}

void MPU9250_Temperature(float *temperature){
    uint8_t raw_temp[2];
    uint16_t temp;

    MPU_ReadTemperature(raw_temp);

    temp = (raw_temp[0] << 8) | raw_temp[1];

    *temperature = (float)(temp / 333.87) + 21.0;
}

void MPU9250_Motion(float *acceleration, float *degrees){
    MPU9250_Accelerometer(acceleration);
    MPU9250_Gyroscope(degrees);
}


