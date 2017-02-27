#include "gpio.h"

void GPIO_Init(void){
    /*GPIO setup for alarm LEDs*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    /*GPIO setup for MPU9250 interrupt pin*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    #ifdef MPU9250_INTERRUPT
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    IntEnable(INT_GPIOC);
    #endif
}

#ifdef MPU9250_INTERRUPT
void GPIO_ClearInterrupt(void){
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
}
#endif


void GPIO_on(uint8_t pins){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1| GPIO_PIN_2| GPIO_PIN_3, pins);
}