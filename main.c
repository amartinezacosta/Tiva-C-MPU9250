#include "mpu.h"
#include "console.h"

#define X   0
#define Y   1
#define Z   2

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    InitConsole();
    init_mpu();

    /*raw data variables*/
    int16_t accelerometer[3];
    int16_t gyroscope[3];
    uint8_t id;

    /*float value conversions*/
    float acceleration[3];
    float angle[3];
    float aRes = 4.0/32768.0;
    float gRes = 500.0/32768.0;

    /*checking connection here*/
    mpu_who_ami(&id);
    if(id == 0x71){
        //we are connected
    }else{
        //connection failed somehow, check connections
        UARTprintf("Connection failed id found: %x", id);
        while(1);
    }

    while(1){
        mpu_read_accelerometer(accelerometer);
        mpu_read_gyro(gyroscope);

        /*UARTprintf("X: %i\n", accelerometer[0]);
        UARTprintf("Y: %i\n", accelerometer[1]);
        UARTprintf("Z: %i\n\n", accelerometer[2]);

        UARTprintf("X angle: %i\n", gyroscope[0]);
        UARTprintf("Y angle: %i\n", gyroscope[1]);
        UARTprintf("Z angle: %i\n\n", gyroscope[2]);*/

        /*converting raw values*/
        acceleration[X] = (float)accelerometer[X]*aRes;
        acceleration[Y] = (float)accelerometer[Y]*aRes;
        acceleration[Z] = (float)accelerometer[Z]*aRes;

        angle[X] = (float)gyroscope[X]*gRes;
        angle[Y] = (float)gyroscope[Y]*gRes;
        angle[Z] = (float)gyroscope[Z]*gRes;

        SysCtlDelay(100);
    }
}
