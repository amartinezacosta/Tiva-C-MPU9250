#ifndef ALARM_H_
#define ALARM_H_

#include "gpio.h"

#define RED_LED         GPIO_PIN_1
#define BLUE_LED        GPIO_PIN_2
#define GREEN_LED       GPIO_PIN_3
#define NONE            0x00

void LED_ArmedAlarm(void);
void LED_FailedAlarm(void);
void Alarms_Init(void);

#endif /* ALARM_H_ */
