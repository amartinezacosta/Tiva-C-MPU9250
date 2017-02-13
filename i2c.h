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

void i2c_write(uint8_t data);
uint32_t i2c_read(void);
void init_i2c(void);

#endif /* I2C_H_ */
