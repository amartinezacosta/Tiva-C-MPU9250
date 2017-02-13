#include "mpu.h"

uint8_t mpu_read(uint8_t address){
    uint8_t data;
    i2c_write(address);
    data = (uint8_t)i2c_read();
    return data;
}

void mpu_write(uint8_t address, uint8_t data){
    i2c_write(address);
    i2c_write(data);
}

void init_mpu(void){
    init_i2c();
    /*CODE INITIALIZATION TAKEN FROM https://github.com/sparkfun/MPU-9250_Breakout/tree/master/Libraries/Arduino */

    // wake up device
    mpu_write(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    SysCtlDelay(100);            // Wait for all registers to reset

    // get stable time source
    mpu_write(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    SysCtlDelay(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    mpu_write(CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    mpu_write(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                 // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
     uint8_t c = mpu_read(GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0;   // Clear self-test bits [7:5]
     c = c & ~0x02;     // Clear Fchoice bits [1:0]
     c = c & ~0x18;     // Clear AFS bits [4:3]
     c = c | 1 << 3;    // Set full scale range for the gyro
    // c =| 0x00;       // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
     mpu_write(GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
     c = mpu_read(ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0;   // Clear self-test bits [7:5]
     c = c & ~0x18;     // Clear AFS bits [4:3]
     c = c | 1 << 3;    // Set full scale range for the accelerometer
     mpu_write(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
     c = mpu_read(ACCEL_CONFIG_2); // get current ACCEL_CONFIG2 register value
     c = c & ~0x0F;     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
     c = c | 0x03;      // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
     mpu_write(ACCEL_CONFIG_2, c); // Write new ACCEL_CONFIG2 register value
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

     // Configure Interrupts and Bypass Enable
     // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
     // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
     // can join the I2C bus and all can be controlled by the Arduino as master
     mpu_write(INT_PIN_CFG, 0x22);
     mpu_write(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
     SysCtlDelay(100);
}

void mpu_read_accelerometer(int16_t *data){
    uint8_t raw_data[6];
    raw_data[0] = mpu_read(ACCEL_XOUT_H);
    raw_data[1] = mpu_read(ACCEL_XOUT_L);
    raw_data[2] = mpu_read(ACCEL_YOUT_H);
    raw_data[3] = mpu_read(ACCEL_YOUT_L);
    raw_data[4] = mpu_read(ACCEL_ZOUT_H);
    raw_data[5] = mpu_read(ACCEL_ZOUT_L);

    data[0] = (raw_data[0] << 8)  | raw_data[1];
    data[1] = (raw_data[2] << 8)  | raw_data[3];
    data[2] = (raw_data[4] << 8)  | raw_data[5];
}

void mpu_read_gyro(int16_t *data){
    uint8_t raw_data[6];
    raw_data[0] = mpu_read(GYRO_XOUT_H);
    raw_data[1] = mpu_read(GYRO_XOUT_L);
    raw_data[2] = mpu_read(GYRO_YOUT_H);
    raw_data[3] = mpu_read(GYRO_YOUT_L);
    raw_data[4] = mpu_read(GYRO_ZOUT_H);
    raw_data[5] = mpu_read(GYRO_ZOUT_L);

    data[0] = (raw_data[0] << 8)  | raw_data[1];
    data[1] = (raw_data[2] << 8)  | raw_data[3];
    data[2] = (raw_data[4] << 8)  | raw_data[5];
}

void mpu_read_temperature(uint16_t *data){
    uint8_t raw_data[2];
    raw_data[0] = mpu_read(TEMP_OUT_H);
    raw_data[1] = mpu_read(TEMP_OUT_L);

    *data = (raw_data[0] << 8) | raw_data[1];
}

void mpu_who_ami(uint8_t *id){
    *id = mpu_read(WHO_AM_I);
}

