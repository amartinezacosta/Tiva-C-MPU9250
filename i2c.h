/*
 * i2c.h
 *
 *  Created on: Feb 8, 2017
 *      Author: alex
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

void I2C_WriteBytes(uint8_t device_address, uint8_t device_register, uint8_t *data, uint8_t lenght);
void I2C_WriteByte(uint8_t slave_address, uint8_t slave_register, uint8_t data);
void I2C_ReadBytes(uint8_t device_address, uint8_t device_register, uint8_t *data, uint8_t lenght);
uint8_t I2C_ReadByte(uint8_t device_address, uint8_t device_register);
void I2C_Init(void);

#endif /* I2C_H_ */
