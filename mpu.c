#include "mpu.h"
#include "mpu9250_rm.h"

/*Self Test Gyroscope and Accelerometer Registers*/
void MPU_WriteSelfTestGyro(uint8_t *data){
    I2C_WriteBytes(MPU9250, SELF_TEST_X_GYRO, data, 3);
}

void MPU_ReadSelfTestGyro(uint8_t *data){
    I2C_ReadBytes(MPU9250, SELF_TEST_X_GYRO, data, 3);
}
void MPU_WriteSelfTestAcce(uint8_t *data){
    I2C_WriteBytes(MPU9250, SELF_TEST_X_ACCEL, data, 3);
}

void MPU_ReadSelfTestAcce(uint8_t *data){
    I2C_WriteBytes(MPU9250, SELF_TEST_X_ACCEL, data, 3);
}

/*Offsets Gyroscope and Accelerometer Registers*/
void MPU_WriteOffsetsGyro(uint8_t *data){
    I2C_WriteBytes(MPU9250, XG_OFFSET_H, data, 6);
}

void MPU_ReadOffsetsGyro(uint8_t *data){
    I2C_ReadBytes(MPU9250, XG_OFFSET_H, data, 6);
}

void MPU_WriteOffsetsAcce(uint8_t *data){
    I2C_WriteBytes(MPU9250, XA_OFFSET_H, data, 6);
}

void MPU_ReadOffsetsAcce(uint8_t *data){
    I2C_ReadBytes(MPU9250, XA_OFFSET_H, data, 6);
}

/*Sample Rate Divider Register*/
void MPU_WriteSampleRateDivider(uint8_t data){
    I2C_WriteByte(MPU9250, SMPLRT_DIV, data);
}

uint8_t MPU_ReadSampleRateDivider(void){
    return I2C_ReadByte(MPU9250, SMPLRT_DIV);
}

/*Configuration Register*/
void MPU_WriteConfiguration(uint8_t data){
    I2C_WriteByte(MPU9250, CONFIG, data);
}

uint8_t MPU_ReadConfiguration(void){
    return I2C_ReadByte(MPU9250, CONFIG);
}

/*Gyroscope Configuration Register*/
void MPU_WriteGyroConfiguration(uint8_t data){
    I2C_WriteByte(MPU9250, GYRO_CONFIG, data);
}

uint8_t MPU_ReadGyroConfiguration(void){
    return I2C_ReadByte(MPU9250, GYRO_CONFIG);
}

/*Accelerometer Configuration Register*/
void MPU_WriteAcceConfiguration(uint8_t data){
    I2C_WriteByte(MPU9250, ACCEL_CONFIG, data);
}

uint8_t MPU_ReadAcceConfiguration(void){
    return I2C_ReadByte(MPU9250, ACCEL_CONFIG);
}

/*Accelerometer Configuration 2 Register*/
void MPU_WriteAcceConfiguration2(uint8_t data){
    I2C_WriteByte(MPU9250, ACCEL_CONFIG_2, data);
}

uint8_t MPU_ReadAcceConfiguration2(void){
    return I2C_ReadByte(MPU9250, ACCEL_CONFIG_2);
}

/*Low Power Accelerometer ODR Control Register*/
void MPU_WriteAcceLowPowerODR(uint8_t data){
    I2C_WriteByte(MPU9250, LP_ACCEL_ODR, data);
}

uint8_t MPU_ReadAcceLowPowerODR(void){
    return I2C_ReadByte(MPU9250, LP_ACCEL_ODR);
}

/*Wake On Motion Threshold Register*/
void MPU_WriteWakeOnMotion(uint8_t data){
    I2C_WriteByte(MPU9250, WOM_THR, data);
}

uint8_t MPU_ReadWakeOnMotion(void){
    return I2C_ReadByte(MPU9250, WOM_THR);
}

/*FIFO Enable Register*/
void MPU_WriteFIFOEnable(uint8_t data){
    I2C_WriteByte(MPU9250, FIFO_EN, data);
}

uint8_t MPU_ReadFIFOEnable(void){
    return I2C_ReadByte(MPU9250, FIFO_EN);
}

/*I2C Master Control Register*/
void MPU_WriteI2CMasterControl(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_MST_CTRL, data);
}

uint8_t MPU_ReadI2CMasterControl(void){
    return I2C_ReadByte(MPU9250, I2C_MST_CTRL);
}

/*I2C Slave 0 Control Registers*/
void MPU_WriteI2CSlave0Addr(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV0_ADDR, data);
}

uint8_t MPU_ReadI2Slave0Addr(void){
    return I2C_ReadByte(MPU9250, SMPLRT_DIV);
}

void MPU_WriteI2CSlave0Reg(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV0_REG, data);
}

uint8_t MPU_ReadI2Slave0Reg(void){
    return I2C_ReadByte(MPU9250, SMPLRT_DIV);
}

void MPU_WriteI2CSlave0Ctrl(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV0_CTRL, data);
}

uint8_t MPU_ReadI2Slave0Ctrl(void){
    return I2C_ReadByte(MPU9250, SMPLRT_DIV);
}

void MPU_WriteI2CSlave0DO(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV0_DO, data);
}

uint8_t MPU_ReadI2Slave0DO(void){
    return I2C_ReadByte(MPU9250, SMPLRT_DIV);
}

/*I2C Slave 1 Control Registers*/
void MPU_WriteI2CSlave1Addr(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV1_ADDR, data);
}

uint8_t MPU_ReadI2Slave1Addr(void){
    return I2C_ReadByte(MPU9250, I2C_SLV1_ADDR);
}

void MPU_WriteI2CSlave1Reg(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV1_REG, data);
}

uint8_t MPU_ReadI2Slave1Reg(void){
    return I2C_ReadByte(MPU9250, I2C_SLV1_REG);
}

void MPU_WriteI2CSlave1Ctrl(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV1_CTRL, data);
}

uint8_t MPU_ReadI2Slave1Ctrl(void){
    return I2C_ReadByte(MPU9250, I2C_SLV1_CTRL);
}

void MPU_WriteI2CSlave1DO(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV1_DO, data);
}

uint8_t MPU_ReadI2Slave1DO(void){
    return I2C_ReadByte(MPU9250, I2C_SLV1_DO);
}

/*I2C Slave 2 Control Registers*/
void MPU_WriteI2CSlave2Addr(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV2_ADDR, data);
}

uint8_t MPU_ReadI2Slave2Addr(void){
    return I2C_ReadByte(MPU9250, I2C_SLV2_ADDR);
}

void MPU_WriteI2CSlave2Reg(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV2_REG, data);
}

uint8_t MPU_ReadI2Slave2Reg(void){
    return I2C_ReadByte(MPU9250, I2C_SLV2_REG);
}

void MPU_WriteI2CSlave2Ctrl(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV2_CTRL, data);
}

uint8_t MPU_ReadI2Slave2Ctrl(void){
    return I2C_ReadByte(MPU9250, I2C_SLV2_CTRL);
}

void MPU_WriteI2CSlave2DO(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV2_DO, data);
}

uint8_t MPU_ReadI2Slave2DO(void){
    return I2C_ReadByte(MPU9250, I2C_SLV2_DO);
}

/*I2C Slave 3 Control Registers*/
void MPU_WriteI2CSlave3Addr(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV3_ADDR, data);
}

uint8_t MPU_ReadI2Slave3Addr(void){
    return I2C_ReadByte(MPU9250, I2C_SLV3_ADDR);
}

void MPU_WriteI2CSlave3Reg(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV3_REG, data);
}

uint8_t MPU_ReadI2Slave3Reg(void){
    return I2C_ReadByte(MPU9250, I2C_SLV3_REG);
}

void MPU_WriteI2CSlave3Ctrl(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV3_CTRL, data);
}

uint8_t MPU_ReadI2Slave3Ctrl(void){
    return I2C_ReadByte(MPU9250, I2C_SLV3_CTRL);
}

void MPU_WriteI2CSlave3DO(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV3_DO, data);
}

uint8_t MPU_ReadI2Slave3DO(void){
    return I2C_ReadByte(MPU9250, I2C_SLV3_DO);
}

/*I2C Slave 4 Control Registers*/
void MPU_WriteI2CSlave4Addr(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV4_ADDR, data);
}

uint8_t MPU_ReadI2Slave4Addr(void){
    return I2C_ReadByte(MPU9250, I2C_SLV4_ADDR);
}

void MPU_WriteI2CSlave4Reg(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV4_REG, data);
}

uint8_t MPU_ReadI2Slave4Reg(void){
    return I2C_ReadByte(MPU9250, I2C_SLV4_REG);
}

void MPU_WriteI2CSlave4Ctrl(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV4_CTRL, data);
}

uint8_t MPU_ReadI2Slave4Ctrl(void){
    return I2C_ReadByte(MPU9250, I2C_SLV4_CTRL);
}

void MPU_WriteI2CSlave4DO(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_SLV4_DO, data);
}

uint8_t MPU_ReadI2Slave4DO(void){
    return I2C_ReadByte(MPU9250, I2C_SLV4_DO);
}

uint8_t MPU_ReadI2Slave4DI(void){
    return I2C_ReadByte(MPU9250, I2C_SLV4_DI);
}

/*I2C Master Status Register*/
uint8_t MPU_ReadI2CMasterStatus(void){
    return I2C_ReadByte(MPU9250, I2C_MST_STATUS);
}

/*INT Pin / Bypass Enable Configuration Register*/
void MPU_WriteINTPinEnable(uint8_t data){
    I2C_WriteByte(MPU9250, INT_PIN_CFG, data);
}

uint8_t MPU_ReadINTPinEnable(void){
    return I2C_ReadByte(MPU9250, INT_PIN_CFG);
}

/*Interrupt Enable Register*/
void MPU_WriteINTEnable(uint8_t data){
    I2C_WriteByte(MPU9250, INT_ENABLE, data);
}
uint8_t MPU_ReadINTEnable(void){
    return I2C_ReadByte(MPU9250, INT_ENABLE);
}

/*Interrupt Status Register*/
uint8_t MPU_ReadINTStatus(void){
    return I2C_ReadByte(MPU9250, INT_STATUS);
}

/*Accelerometer, Gyroscope and Temperature Measurements Registers*/
void MPU_ReadAccelerometer(uint8_t *data){
    I2C_ReadBytes(MPU9250, ACCEL_XOUT_H, data, 6);
}

void MPU_ReadGyroscope(uint8_t *data){
    I2C_ReadBytes(MPU9250, GYRO_XOUT_H, data, 6);
}

void MPU_ReadTemperature(uint8_t *data){
    I2C_ReadBytes(MPU9250, TEMP_OUT_H, data, 2);
}

/*External Sensor Data Registers*/
void MPU_ReadExternalSensors(uint8_t *data, uint8_t size){
    I2C_ReadBytes(MPU9250, EXT_SENS_DATA_00, data, size);
}

/*I2C Master Delay Control Register*/
void MPU_WriteI2CMasterDelayCtl(uint8_t data){
    I2C_WriteByte(MPU9250, I2C_MST_DELAY_CTRL, data);
}

uint8_t MPU_ReadI2CMasterDelayCtl(void){
    return I2C_ReadByte(MPU9250, I2C_MST_DELAY_CTRL);
}

/*Signal Path Reset Register*/
void MPU_WriteSignalPathReset(uint8_t data){
    I2C_WriteByte(MPU9250, SIGNAL_PATH_RESET, data);
}

uint8_t MPU_ReadSignalPathReset(void){
    return I2C_ReadByte(MPU9250, SIGNAL_PATH_RESET);
}

/*Accelerometer Interrupt Control Register*/
void MPU_WriteAcceIntControl(uint8_t data){
    I2C_WriteByte(MPU9250, MOT_DETECT_CTRL, data);
}

uint8_t MPU_ReadAcceIntControl(void){
    return I2C_ReadByte(MPU9250, MOT_DETECT_CTRL);
}

/*User Control Register*/
void MPU_WriteUserControl(uint8_t data){
    I2C_WriteByte(MPU9250, USER_CTRL, data);
}

uint8_t MPU_ReadUserControl(void){
    return I2C_ReadByte(MPU9250, USER_CTRL);
}

/*Power Management 1 Register*/
void MPU_WritePowerManagement1(uint8_t data){
    I2C_WriteByte(MPU9250, PWR_MGMT_1, data);
}

uint8_t MPU_ReadPowerManagement1(void){
    return I2C_ReadByte(MPU9250, PWR_MGMT_1);
}

/*Power Management 2 Register*/
void MPU_WritePowerManagement2(uint8_t data){
    I2C_WriteByte(MPU9250, PWR_MGMT_2, data);
}

uint8_t MPU_ReadPowerManagement2(void){
    return I2C_ReadByte(MPU9250, PWR_MGMT_2);
}

/*FIFO Count Registers*/
void MPU_ReadFIFOCount(uint8_t *data){
    I2C_ReadBytes(MPU9250, FIFO_COUNTH, data, 2);
}

/*FIFO Read Write Register*/
uint8_t MPU_ReadFIFO(void){
    return I2C_ReadByte(MPU9250, FIFO_R_W);
}

void MPU_WriteFIFO(uint8_t data){
    I2C_WriteByte(MPU9250, FIFO_R_W, data);
}

/*Who Am I Register*/
uint8_t MPU_WhoAmI(void){
    return I2C_ReadByte(MPU9250, WHO_AM_I);
}



