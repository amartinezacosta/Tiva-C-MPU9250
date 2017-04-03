#include "mpu9250.h"
#include "console.h"
#include "alarm.h"

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    InitConsole();
    Alarms_Init();
    I2C_Init();

    uint8_t id;

    /*Checking connection here*/
    id = MPU_WhoAmI();
    if(id == 0x71){
        /*We are connected set armed alarm*/
        LED_ArmedAlarm();
    }else{
        /*connection failed somehow, set failed alarm. Check connections*/
        UARTprintf("Connection failed id found: %x", id);
        LED_FailedAlarm();
        while(1);
    }

    MPU_WritePowerManagement1(CLKSEL_1);
    MPU_WriteGyroConfiguration(GYRO_FS_SELECT_250);
    MPU_WriteAcceConfiguration(ACCE_FS_SELECT_2G);

    float acceleration_bias[3];
    float degree_bias[3];

    MPU9250_calibrate(acceleration_bias, degree_bias);

    float acceleration[3];
    float degrees[3];
    float temperature;

    while(1){
        MPU9250_Motion(acceleration, degrees);
        MPU9250_Temperature(&temperature);

        acceleration[0] -= acceleration_bias[0];
        acceleration[1] -= acceleration_bias[1];
        acceleration[2] -= acceleration_bias[2];

        degrees[0] -= degree_bias[0];
        degrees[1] -= degree_bias[1];
        degrees[2] -= degree_bias[2];

        SysCtlDelay(1000);
    }
}
