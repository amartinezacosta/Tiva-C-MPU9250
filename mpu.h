#ifndef MPU_H_
#define MPU_H_

#include "i2c.h"

/*Parameters that can be passed to the Configuration Register*/
#define FIFO_ENABLE_BIT         0x40
#define EXT_SYNC_DISABLED       0x00
#define EXT_SYNC_TEMP           0x08
#define EXT_SYNC_GYRO_XOUT      0x10
#define EXT_SYNC_GYRO_YOUT      0x18
#define EXT_SYNC_GYRO_ZOUT      0x20
#define EXT_SYNC_ACCE_XOUT      0x28
#define EXT_SYNC_ACCE_YOUT      0x21
#define EXT_SYNC_ACCE_ZOUT      0x38
#define DLPF_CFG_0              0x00    //Gyroscope: 250 Hz Bandwidth,  0.97 Delay(ms),  8 KHz Fs .... Temperature Sensor: 4000 Hz Bandwidth, .04 Delay(ms)
#define DLPF_CFG_1              0x01    //Gyroscope: 184 Hz Bandwidth,  2.9 Delay(ms),   1 KHz Fs .... Temperature Sensor: 188 Hz Bandwidth,  1.9 Delay(ms)
#define DLPF_CFG_2              0x02    //Gyroscope: 250 Hz Bandwidth,  3.9 Delay(ms),   1 KHz Fs .... Temperature Sensor: 98 Hz Bandwidth,   2.8 Delay(ms)
#define DLPF_CFG_3              0x03    //Gyroscope: 92 Hz Bandwidth,   5.9 Delay(ms),   1 KHz Fs .... Temperature Sensor: 42 Hz Bandwidth,   4.8 Delay(ms)
#define DLPF_CFG_4              0x04    //Gyroscope: 41 Hz Bandwidth,   9.9 Delay(ms),   1 KHz Fs .... Temperature Sensor: 20 Hz Bandwidth,   8.3 Delay(ms)
#define DLPF_CFG_5              0x05    //Gyroscope: 20 Hz Bandwidth,   17.85 Delay(ms), 1 KHz Fs .... Temperature Sensor: 10 Hz Bandwidth,   13.4 Delay(ms)
#define DLPF_CFG_6              0x06    //Gyroscope: 10 Hz Bandwidth,   33.84 Delay(ms), 1 KHz Fs .... Temperature Sensor: 5 Hz Bandwidth,    18.6 Delay(ms)
#define DLPF_CFG_7              0x07    //Gyroscope: 3600 Hz Bandwidth, 0.17 Delay(ms),  8 KHz Fs .... Temperature Sensor: 4000 Hz Bandwidth, 0.04 Delay(ms)

/*Parameters that can be passed to the Gyroscope Configuration Register*/
#define X_GYRO_TEST             0x80
#define Y_GYRO_TEST             0x40
#define Z_GYRO_TEST             0x20
#define GYRO_FS_SELECT_250      0x00    //+250 dps
#define GYRO_FS_SELECT_500      0x08    //+500 dps
#define GYRO_FS_SELECT_1000     0x10    //+1000 dps
#define GYRO_FS_SELECT_2000     0x18    //+2000 dps
#define FCHOICE_B               0x03

/*Parameters that can be passed to the Accelerometer Configuration Register*/
#define X_ACCE_TEST             0x80
#define Y_ACCE_TEST             0x40
#define Z_ACCE_TEST             0x20
#define ACCE_FS_SELECT_2G       0x00    //+-2 g
#define ACCE_FS_SELECT_4G       0x08    //+-4 g
#define ACCE_FS_SELECT_8G       0x10    //+-8 g
#define ACCE_FS_SELECT_16G      0x18    //+-16 g

/*Parameters that can be passed to the Accelerometer Configuration 2 Register*/
#define ACCEL_CHOICE_B      0x04
#define A_DLPF_CFG_0        0x00    //Accelerometer: 460 Hz Bandwidth, Delay(ms) 1.94,  Noise Density(ug/rtHz) 250, Rate 1 KHz
#define A_DLPF_CFG_1        0x01    //Accelerometer: 184 Hz Bandwidth, Delay(ms) 5.80,  Noise Density(ug/rtHz) 250, Rate 1 KHz
#define A_DLPF_CFG_2        0x02    //Accelerometer: 92 Hz Bandwidth,  Delay(ms) 7.80,  Noise Density(ug/rtHz) 250, Rate 1 KHz
#define A_DLPF_CFG_3        0x03    //Accelerometer: 41 Hz Bandwidth,  Delay(ms) 11.80, Noise Density(ug/rtHz) 250, Rate 1 KHz
#define A_DLPF_CFG_4        0x04    //Accelerometer: 20 Hz Bandwidth,  Delay(ms) 19.80, Noise Density(ug/rtHz) 250, Rate 1 KHz
#define A_DLPF_CFG_5        0x05    //Accelerometer: 10 Hz Bandwidth,  Delay(ms) 35.70, Noise Density(ug/rtHz) 250, Rate 1 KHz
#define A_DLPF_CFG_6        0x06    //Accelerometer: 5 Hz Bandwidth,   Delay(ms) 66.96, Noise Density(ug/rtHz) 250, Rate 1 KHz
#define A_DLPF_CFG_7        0x07    //Accelerometer: 460 Hz Bandwidth, Delay(ms) 1.94,  Noise Density(ug/rtHz) 250, Rate 1 KHz

/*Parameters that can be passed to the Low Power Accelerometer ODR Control Register*/
#define LPOSC_CLKSEL_0      0x00    //0.24 Hz ODR,  1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_1      0x01    //0.49 Hz ODR,  1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_2      0x02    //0.98 Hz ODR,  1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_3      0x03    //1.95 Hz ODR,  1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_4      0x04    //3.91 Hz ODR,  1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_5      0x05    //7.81 Hz ODR,  1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_6      0x06    //15.63 Hz ODR, 1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_7      0x07    //31.25 Hz ODR, 1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_8      0x08    //62.50 Hz ODR, 1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_9      0x09    //125 Hz ODR,   1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_10     0x0A    //250 Hz ODR,   1.1 KHz Bandwidth, Delay(ms) 1
#define LPOSC_CLKSEL_11     0x0B    //500 Hz ODR,   1.1 KHz Bandwidth, Delay(ms) 1

/*Parameters that can be passed to the FIFO Enable Register*/
#define TEMP_OUT            0x80
#define GYRO_XOUT           0x40
#define GYRO_YOUT           0x20
#define GYRO_ZOUT           0x10
#define ACCEL               0x08
#define SLV_2               0x04
#define SLV_1               0x02
#define SLV_0               0x01

/*Parameters that can be passed to the INT Pin/Bypass Enable Configuration Register*/
#define ACTL                0x80    //The Logic level for INT pin is active low
#define OPEN                0x40    //INT pin configured as open drain
#define LATCH_INT_EN        0x20    //INT pin level held until interrupt status is cleared
#define INT_ANDRYD_2CLEAR   0x10    //Interrupt status is cleared if any read operation is performed
#define ACTL_FSYNC          0x08    //The logic level for the FSYNC pin as an interrupt is active low
#define FSYNC_INT_MODE_EN   0x04    //Enables the FSYNC pin to be used as an interrupt
#define BYPASS_EN           0x02    //When asserted the I2C master interface pins, will go into 'bypass mode' when the I2C master interface is disabled

/*Parameters that can be passed to the Interrupt Enable Register*/
#define WOM_EN              0x40    //Enable interrupt for wake on motion to propagate to interrupt pin
#define FIFO_OVERFLOW_EN    0x10    //Enable interrupt for FIFO overflow to propagate to interrupt pin
#define FSYNC_INT_EN        0x08    //Enable FSYNC interrupt to propagate to interrupt pin
#define RAW_RDY_EN          0x01    //Enable raw sensor data ready interrupt to propagate to interrupt pin

/*Interrupt status masks*/
#define WOM_INT             0x40    //Wake on motion interrupt occurred
#define FIFO_OVERFLOW_INT   0x10    //FIFO overflow interrupt occurred
#define FSYNC_INT           0x08    //FSYNC interrupt occurred
#define RAW_DATA_RDY_INT    0x01    //Sensor Register raw data sensors

/*Parameters that can be passed to the Signal Path Reset Register*/
#define GYRO_RST            0x04
#define ACCEL_RST           0x02
#define TEMP_RST            0x01

/*Parameters that can be passed to the Accelerometer Interrupt Control Register*/
#define ACCEL_INT_EN        0x80    //Enable wake on motion detection logic
#define ACCEL_INT_MODE_0    0x40    //Compare the current sample with the previous sample
#define ACCEL_INT_MODE_1    0x00    //No used

/*Parameters that can be passed to the User Control Register*/
#define FIFO_ENABLE         0x40
#define I2C_MST_EN          0x20
#define I2C_IF_DIS          0x10
#define FIFO_RST            0x04
#define I2C_MST_RST         0x02
#define SIG_COND_RST        0x01

/*Parameters that can be passed to the Power Management 1 Register*/
#define H_RESET             0x80    //Reset the internal registers and restores default settings
#define SLEEP               0x40    //When set, the chip is set to sleep mode
#define CYCLE               0x20    //When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking a single sample at a rate determined by LP_ACCEL_ODR
#define GYRO_STANDBY        0x10    //The gyroscope drive and PLL circuitry are enabled, but the sense paths are disabled
#define PD_PTAT             0x08    //Power down internal PTAT voltage generator and PTAT ADC
#define CLKSEL_0            0x00    //Internal 20MHz oscillator
#define CLKSEL_1            0x01    //Auto-selects the best available clock source - PLL if ready, else use the internal oscillator
#define CLKSEL_2            0x02    //Auto-selects the best available clock source - PLL if ready, else use the internal oscillator
#define CLKSEL_3            0x03    //Auto-selects the best available clock source - PLL if ready, else use the internal oscillator
#define CLKSEL_4            0x04    //Auto-selects the best available clock source - PLL if ready, else use the internal oscillator
#define CLKSEL_5            0x05    //Auto-selects the best available clock source - PLL if ready, else use the internal oscillator
#define CLKSEL_6            0x06    //Internal 20MHz oscillator
#define CLKSEL_7            0x07    //Stops the clock and keeps timing generator in reset

/*Parameters that can be passed to the Power Management 2 Register*/
#define DISABLE_XA          0x20
#define DISABLE_YA          0x10
#define DISABLE_ZA          0x08
#define DISABLE_XG          0x04
#define DISABLE_YG          0x02
#define DISABLE_ZG          0x01


/*Self Test Gyroscope and Accelerometer Registers*/
void MPU_WriteSelfTestGyro(uint8_t *data);
void MPU_ReadSelfTestGyro(uint8_t *data);
void MPU_WriteSelfTestAcce(uint8_t *data);
void MPU_ReadSelfTestAcce(uint8_t *data);

/*Offsets Gyroscope and Accelerometer Registers*/
void MPU_WriteOffsetsGyro(uint8_t *data);
void MPU_ReadOffsetsGyro(uint8_t *data);
void MPU_WriteOffsetsAcce(uint8_t *data);
void MPU_ReadOffsetsAcce(uint8_t *data);

/*Sample Rate Divider Register*/
void MPU_WriteSampleRateDivider(uint8_t data);
uint8_t MPU_ReadSampleRateDivider(void);

/*Configuration Register*/
void MPU_WriteConfiguration(uint8_t data);
uint8_t MPU_ReadConfiguration(void);

/*Gyroscope Configuration Register*/
void MPU_WriteGyroConfiguration(uint8_t data);
uint8_t MPU_ReadGyroConfiguration(void);

/*Accelerometer Configuration Register*/
void MPU_WriteAcceConfiguration(uint8_t data);
uint8_t MPU_ReadAcceConfiguration(void);

/*Accelerometer Configuration 2 Register*/
void MPU_WriteAcceConfiguration2(uint8_t data);
uint8_t MPU_ReadAcceConfiguration2(void);

/*Low Power Accelerometer ODR Control Register*/
void MPU_WriteAcceLowPowerODR(uint8_t data);
uint8_t MPU_ReadAcceLowPowerODR(void);

/*Wake On Motion Threshold Register*/
void MPU_WriteWakeOnMotion(uint8_t data);
uint8_t MPU_ReadWakeOnMotion(void);

/*FIFO Enable Register*/
void MPU_WriteFIFOEnable(uint8_t data);
uint8_t MPU_ReadFIFOEnable(void);

/*I2C Master Control Register*/
void MPU_WriteI2CMasterControl(uint8_t data);
uint8_t MPU_ReadI2CMasterControl(void);

/*I2C Slave 0 Control Registers*/
void MPU_WriteI2CSlave0Addr(uint8_t data);
uint8_t MPU_ReadI2Slave0Addr(void);
void MPU_WriteI2CSlave0Reg(uint8_t data);
uint8_t MPU_ReadI2Slave0Reg(void);
void MPU_WriteI2CSlave0Ctrl(uint8_t data);
uint8_t MPU_ReadI2Slave0Ctrl(void);
void MPU_WriteI2CSlave0DO(uint8_t data);
uint8_t MPU_ReadI2Slave0DO(void);

/*I2C Slave 1 Control Registers*/
void MPU_WriteI2CSlave1Addr(uint8_t data);
uint8_t MPU_ReadI2Slave1Addr(void);
void MPU_WriteI2CSlave1Reg(uint8_t data);
uint8_t MPU_ReadI2Slave1Reg(void);
void MPU_WriteI2CSlave1Ctrl(uint8_t data);
uint8_t MPU_ReadI2Slave1Ctrl(void);
void MPU_WriteI2CSlave1DO(uint8_t data);
uint8_t MPU_ReadI2Slave1DO(void);

/*I2C Slave 2 Control Registers*/
void MPU_WriteI2CSlave2Addr(uint8_t data);
uint8_t MPU_ReadI2Slave2Addr(void);
void MPU_WriteI2CSlave2Reg(uint8_t data);
uint8_t MPU_ReadI2Slave2Reg(void);
void MPU_WriteI2CSlave2Ctrl(uint8_t data);
uint8_t MPU_ReadI2Slave2Ctrl(void);
void MPU_WriteI2CSlave2DO(uint8_t data);
uint8_t MPU_ReadI2Slave2DO(void);

/*I2C Slave 3 Control Registers*/
void MPU_WriteI2CSlave3Addr(uint8_t data);
uint8_t MPU_ReadI2Slave3Addr(void);
void MPU_WriteI2CSlave3Reg(uint8_t data);
uint8_t MPU_ReadI2Slave3Reg(void);
void MPU_WriteI2CSlave3Ctrl(uint8_t data);
uint8_t MPU_ReadI2Slave3Ctrl(void);
void MPU_WriteI2CSlave3DO(uint8_t data);
uint8_t MPU_ReadI2Slave3DO(void);

/*I2C Slave 4 Control Registers*/
void MPU_WriteI2CSlave4Addr(uint8_t data);
uint8_t MPU_ReadI2Slave4Addr(void);
void MPU_WriteI2CSlave4Reg(uint8_t data);
uint8_t MPU_ReadI2Slave4Reg(void);
void MPU_WriteI2CSlave4Ctrl(uint8_t data);
uint8_t MPU_ReadI2Slave4Ctrl(void);
void MPU_WriteI2CSlave4DO(uint8_t data);
uint8_t MPU_ReadI2Slave4DO(void);
uint8_t MPU_ReadI2Slave4DI(void);

/*I2C Master Status Register*/
uint8_t MPU_ReadI2CMasterStatus(void);

/*INT Pin / Bypass Enable Configuration Register*/
void MPU_WriteINTPinEnable(uint8_t data);
uint8_t MPU_ReadINTPinEnable(void);

/*Interrupt Enable Register*/
void MPU_WriteINTEnable(uint8_t data);
uint8_t MPU_ReadINTEnable(void);

/*Interrupt Status Register*/
uint8_t MPU_ReadINTStatus(void);

/*Accelerometer, Gyroscope and Temperature Measurements Registers*/
void MPU_ReadAccelerometer(uint8_t *data);
void MPU_ReadGyroscope(uint8_t *data);
void MPU_ReadTemperature(uint8_t *data);

/*External Sensor Data Registers*/
void MPU_ReadExternalSensors(uint8_t *data, uint8_t size);

/*I2C Master Delay Control Register*/
void MPU_WriteI2CMasterDelayCtl(uint8_t data);
uint8_t MPU_ReadI2CMasterDelayCtl(void);

/*Signal Path Reset Register*/
void MPU_WriteSignalPathReset(uint8_t data);
uint8_t MPU_ReadSignalPathReset(void);

/*Accelerometer Interrupt Control Register*/
void MPU_WriteAcceIntControl(uint8_t data);
uint8_t MPU_ReadAcceIntControl(void);

/*User Control Register*/
void MPU_WriteAcceIntControl(uint8_t data);
uint8_t MPU_ReadAcceIntControl(void);

/*Power Management 1 Register*/
void MPU_WritePowerManagement1(uint8_t data);
uint8_t MPU_ReadPowerManagement1(void);

/*Power Management 2 Register*/
void MPU_WritePowerManagement2(uint8_t data);
uint8_t MPU_ReadPowerManagement2(void);

/*FIFO Count Registers*/
void MPU_ReadFIFOCount(uint8_t *data);

/*FIFO Read Write Register*/
uint8_t MPU_ReadFIFO(void);
void MPU_WriteFIFO(uint8_t data);

/*Who Am I Register*/
uint8_t MPU_WhoAmI(void);

#endif /* MPU_H_ */
