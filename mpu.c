#include "mpu.h"

float aRes;
float gRes;
uint8_t aScale = 0;
uint8_t gScale = 0;

void MPU9250_SetAccelerometerResolution(uint8_t resolution){
    switch(resolution){
    case 0:
        aRes = 2.0 / 32768.0;
        aScale = 0;
        break;
    case 1:
        aRes = 4.0 / 32768.0;
        aScale = 1;
        break;
    case 2:
        aRes = 8.0 / 32768.0;
        aScale = 2;
        break;
    case 3:
        aRes = 16.0 / 32768.0;
        aScale = 3;
        break;
    default:
        aRes = 2.0 / 32768.0;
        aScale = 0;
        break;
    }
}

void MPU9250_SetGyroscopeResolution(uint8_t resolution){
    switch(resolution){
       case 0:
           gRes = 250.0 / 32768.0;
           gScale = 0;
           break;
       case 1:
           gRes = 500.0 / 32768.0;
           gScale = 1;
           break;
       case 2:
           gRes = 1000.0 / 32768.0;
           gScale = 2;
           break;
       case 3:
           gRes = 2000.0 / 32768.0;
           gScale = 3;
           break;
       default:
           gRes = 250.0 / 32768.0;
           gScale = 0;
           break;
       }
}

void MPU9250_Calibrate(void){
    uint8_t data[12];
    uint16_t fifo_count, packet_count, i;
    int32_t acceleration_bias[3] = {0, 0, 0};
    int32_t gyroscope_bias[3] = {0, 0, 0};

    I2C_WriteByte(MPU9250, PWR_MGMT_1, 0x80);

    I2C_WriteByte(MPU9250, PWR_MGMT_1, 0x01);
    I2C_WriteByte(MPU9250, PWR_MGMT_2, 0x00);

    I2C_WriteByte(MPU9250, INT_ENABLE, 0x00);
    I2C_WriteByte(MPU9250, FIFO_EN, 0x00);
    I2C_WriteByte(MPU9250, PWR_MGMT_1, 0x00);
    I2C_WriteByte(MPU9250, I2C_MST_CTRL, 0x00);
    I2C_WriteByte(MPU9250, USER_CTRL, 0x00);
    I2C_WriteByte(MPU9250, USER_CTRL, 0x0C);

    I2C_WriteByte(MPU9250, CONFIG, 0x01);
    I2C_WriteByte(MPU9250, SMPLRT_DIV, 0x00);
    I2C_WriteByte(MPU9250, GYRO_CONFIG, 0x00);
    I2C_WriteByte(MPU9250, ACCEL_CONFIG, 0x00);

    I2C_WriteByte(MPU9250, USER_CTRL, 0x40);
    I2C_WriteByte(MPU9250, FIFO_EN, 0x78);

    I2C_WriteByte(MPU9250, FIFO_EN, 0x00);
    data[0] = I2C_ReadByte(MPU9250, FIFO_COUNTH);
    fifo_count = (data[0] << 8) | data[1];
    packet_count = fifo_count / 12;

    int16_t a_temp[3] = {0, 0, 0};
    int16_t g_temp[3] = {0, 0, 0};

    for(i = 0; i < packet_count; i++){
        I2C_ReadBytes(MPU9250, FIFO_R_W, data, 12);
        a_temp[0] = (data[0] << 8) | data[1];
        a_temp[1] = (data[2] << 8) | data[3];
        a_temp[2] = (data[4] << 8) | data[5];
        g_temp[0] = (data[6] << 8) | data[7];
        g_temp[1] = (data[8] << 8) | data[9];
        g_temp[2] = (data[10] << 8) | data[11];

        acceleration_bias[0] += a_temp[0];
        acceleration_bias[1] += a_temp[1];
        acceleration_bias[2] += a_temp[2];

        gyroscope_bias[0] += g_temp[0];
        gyroscope_bias[1] += g_temp[1];
        gyroscope_bias[2] += g_temp[2];
    }

    acceleration_bias[0] /= packet_count;
    acceleration_bias[1] /= packet_count;
    acceleration_bias[2] /= packet_count;

    gyroscope_bias[0] /= packet_count;
    gyroscope_bias[1] /= packet_count;
    gyroscope_bias[2] /= packet_count;

    if( acceleration_bias[2] > 0){
        acceleration_bias[2] -= 16384;
    }

    data[0] = (-gyroscope_bias[0]/4 >> 8)   & 0xFF;
    data[1] = (-gyroscope_bias[0]/4)        & 0xFF;
    data[2] = (-gyroscope_bias[1]/4 >> 8)   & 0xFF;
    data[3] = (-gyroscope_bias[1]/4)        & 0xFF;
    data[4] = (-gyroscope_bias[2]/4 >> 8)   & 0xFF;
    data[5] = (-gyroscope_bias[2]/4)        & 0xFF;

    I2C_WriteBytes(MPU9250, XG_OFFSET_H, data, 6);

    int32_t a_reg_bias[3] = {0, 0, 0};
    I2C_ReadBytes(MPU9250, XA_OFFSET_H, data, 2);
    a_reg_bias[0] = (data[0] << 8) | data[1];
    I2C_ReadBytes(MPU9250, YA_OFFSET_H, data, 2);
    a_reg_bias[1] = (data[0] << 8) | data[1];
    I2C_ReadBytes(MPU9250, ZA_OFFSET_H, data, 2);
    a_reg_bias[2] = (data[0] << 8) | data[1];

    uint32_t mask = 1uL;
    uint8_t mask_bit[3] = {0, 0, 0};

    for(i = 0; i < 3; i++) {
        if((a_reg_bias[i] & mask)) mask_bit[i] = 0x01;
    }

    a_reg_bias[0] -= (acceleration_bias[0]/8);
    a_reg_bias[1] -= (acceleration_bias[1]/8);
    a_reg_bias[2] -= (acceleration_bias[2]/8);

    data[0] = (a_reg_bias[0]/4 >> 8)   & 0xFF;
    data[1] = (a_reg_bias[0]/4)        & 0xFF;
    data[1] = data[1] | mask_bit[0];
    data[2] = (a_reg_bias[1]/4 >> 8)   & 0xFF;
    data[3] = (a_reg_bias[1]/4)        & 0xFF;
    data[3] = data[3] | mask_bit[1];
    data[4] = (a_reg_bias[2]/4 >> 8)   & 0xFF;
    data[5] = (a_reg_bias[2]/4)        & 0xFF;
    data[5] = data[5] | mask_bit[2];

    I2C_WriteBytes(MPU9250, XA_OFFSET_H, data, 6);


}

void MPU9250_Init(uint8_t acelerometer_resolution, uint8_t gyroscope_resolution){
    I2C_Init();
    MPU9250_Calibrate();
    MPU9250_SetAccelerometerResolution(acelerometer_resolution);
    MPU9250_SetGyroscopeResolution(gyroscope_resolution);

    #ifdef MPU9250_INTERRUPT
    I2C_WriteByte(MPU9250, PWR_MGMT_1, 0x00);
    I2C_WriteByte(MPU9250, PWR_MGMT_2, 0x07);
    I2C_WriteByte(MPU9250, ACCEL_CONFIG_2, 0x09);
    I2C_WriteByte(MPU9250, INT_ENABLE, 0x40);
    I2C_WriteByte(MPU9250, MOT_DETECT_CTRL, 0xA0);
    I2C_WriteByte(MPU9250, WOM_THR, 0xFF);
    I2C_WriteByte(MPU9250, LP_ACCEL_ODR, 0x07);
    I2C_WriteByte(MPU9250, PWR_MGMT_1, 0x10);
    #endif

    I2C_WriteByte(MPU9250, PWR_MGMT_1, 0x00);
    I2C_WriteByte(MPU9250, PWR_MGMT_1, 0x01);
    I2C_WriteByte(MPU9250, CONFIG, 0x03);
    I2C_WriteByte(MPU9250, SMPLRT_DIV, 0x04);
    I2C_WriteByte(MPU9250, GYRO_CONFIG, 0x00);
    I2C_WriteByte(MPU9250, ACCEL_CONFIG, 0x00);
    I2C_WriteByte(MPU9250, ACCEL_CONFIG_2, 0x03);
    I2C_WriteByte(MPU9250, INT_PIN_CFG, 0x22);
    I2C_WriteByte(MPU9250, INT_ENABLE, 0x01);
}

void MPU9250_AccelerometerRead(float *accelerations){
    int16_t data[3];
    uint8_t raw_data[6];
    I2C_ReadBytes(MPU9250, ACCEL_XOUT_H,  raw_data, 6);

    data[0] = (raw_data[0] << 8)  | raw_data[1];
    data[1] = (raw_data[2] << 8)  | raw_data[3];
    data[2] = (raw_data[4] << 8)  | raw_data[5];

    accelerations[0] = data[0] * aRes;
    accelerations[1] = data[1] * aRes;
    accelerations[2] = data[2] * aRes;
}

void MPU9250_GyroscopeRead(float *angles){
    int16_t data[3];
    uint8_t raw_data[6];
    I2C_ReadBytes(MPU9250, GYRO_XOUT_H, raw_data, 6);

    data[0] = (raw_data[0] << 8)  | raw_data[1];
    data[1] = (raw_data[2] << 8)  | raw_data[3];
    data[2] = (raw_data[4] << 8)  | raw_data[5];

    angles[0] = data[0] * gRes;
    angles[1] = data[1] * gRes;
    angles[2] = data[2] * gRes;
}

void MPU9250_TemperatureRead(float *temperature){
    uint16_t data;
    uint8_t raw_data[2];
    I2C_ReadBytes(MPU9250, TEMP_OUT_H, raw_data, 2);

    data = (raw_data[0] << 8) | raw_data[1];

    *temperature = data;
}

#ifdef MPU9250_INTERRUPT
void MPU9250_InterruptHandler(void){
    GPIO_ClearInterrupt();
}
#endif

void MPU9250_WhoAmI(uint8_t *id){
    *id = I2C_ReadByte(MPU9250, WHO_AM_I);
}

