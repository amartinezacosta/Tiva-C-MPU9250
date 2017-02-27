#include "mpu.h"
#include "console.h"
#include "alarm.h"

#define X   0
#define Y   1
#define Z   2

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    InitConsole();
    MPU9250_Init(0, 0);
    Alarms_Init();
    #ifdef MPU9250_INTERRUPT
    IntMasterEnable();
    #endif

    uint8_t id;

    /*Checking connection here*/
    MPU9250_WhoAmI(&id);
    if(id == 0x71){
        /*We are connected set armed alarm*/
        LED_ArmedAlarm();
    }else{
        /*connection failed somehow, set failed alarm. Check connections*/
        UARTprintf("Connection failed id found: %x", id);
        LED_FailedAlarm();
        while(1);
    }

    float accelerations[3];
    float angles[3];

    MPU9250_GyroscopeRead(angles);
    MPU9250_AccelerometerRead(accelerations);

    while(1){

    }
}
