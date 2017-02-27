#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

//#define MPU9250_INTERRUPT

void GPIO_Init(void);
void GPIO_on(uint8_t pins);

#ifdef MPU9250_INTERRUPT
void GPIO_ClearInterrupt(void);
#endif

#endif /* GPIO_H_ */
