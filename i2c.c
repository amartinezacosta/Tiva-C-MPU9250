#include "i2c.h"

void init_i2c(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
}


void i2c_write(uint8_t data){
    I2CMasterSlaveAddrSet(I2C0_BASE, 0x68, false);
    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    //wait for master i2c to transmit data
    while(I2CMasterBusy(I2C0_BASE)){

    }
}

uint32_t i2c_read(void){
    uint32_t data;
    I2CMasterSlaveAddrSet(I2C0_BASE, 0x68, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(I2C0_BASE)){

    }

    data = I2CMasterDataGet(I2C0_BASE);
    return data;
}
