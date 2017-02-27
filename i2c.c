#include "i2c.h"

void I2C_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)){

    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
}

void I2C_WriteBytes(uint8_t device_address, uint8_t device_register, uint8_t *data, uint8_t lenght){
    I2CMasterSlaveAddrSet(I2C0_BASE, device_address, false);
    I2CMasterDataPut(I2C0_BASE, device_register);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    while(I2CMasterBusy(I2C0_BASE));

    uint8_t i;
    for(i = 0; i < lenght - 1; i++){
        I2CMasterDataPut(I2C0_BASE, data[i]);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(I2CMasterBusy(I2C0_BASE));
    }

    I2CMasterDataPut(I2C0_BASE, data[lenght - 1]);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    while(I2CMasterBusy(I2C0_BASE));
}

void I2C_WriteByte(uint8_t device_address, uint8_t device_register, uint8_t data){
    I2C_WriteBytes(device_address, device_register, &data, 1);
}


void I2C_ReadBytes(uint8_t device_address, uint8_t device_register, uint8_t *data, uint8_t lenght){
    uint8_t i;
    I2CMasterSlaveAddrSet(I2C0_BASE, device_address, false);
    I2CMasterDataPut(I2C0_BASE, device_register);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, device_address, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    while(I2CMasterBusy(I2C0_BASE));

    data[0] = I2CMasterDataGet(I2C0_BASE);

    for(i = 0; i < lenght; i++){
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C0_BASE));
        data[i] = I2CMasterDataGet(I2C0_BASE);
    }

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
    data[lenght - 1] = I2CMasterDataGet(I2C0_BASE);
}


uint8_t I2C_ReadByte(uint8_t device_address, uint8_t device_register){
    I2CMasterSlaveAddrSet(I2C0_BASE, device_address, false);
    I2CMasterDataPut(I2C0_BASE, device_register);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, device_address, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    while(I2CMasterBusy(I2C0_BASE));

    return I2CMasterDataGet(I2C0_BASE);
}


