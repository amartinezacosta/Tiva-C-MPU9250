#include "alarm.h"


void Alarms_Init(void){
    GPIO_Init();
}

void LED_ArmedAlarm(void){
    uint8_t i;
    for(i = 0; i < 8; i++){
        GPIO_on(BLUE_LED);
        SysCtlDelay(1000000);
        GPIO_on(NONE);
        SysCtlDelay(1000000);
    }
}


void LED_FailedAlarm(void){
    GPIO_on(RED_LED);
}
