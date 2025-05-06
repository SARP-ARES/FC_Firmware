#include "mbed.h"

#ifndef _MOTOR_DATA_
#define _MOTOR_DATA_

#pragma pack push(push, 1) // remove memory padding
struct MotorData {
    float delta1_deg;       // 4 
    float delta2_deg;       // 4
    float pwm_motor1;       // 4
    float pwm_motor2;       // 4
};
#pragma pack(pop)

#endif // _MOTOR_DATA_