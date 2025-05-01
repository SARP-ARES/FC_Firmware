#include "mbed.h"
#include <string>
#ifndef MOTOR_PACKET_H
#define MOTOR_PACKET_H

#pragma pack(push, 1) // remove padding to ensure correct memory layout
struct MotorPacket {
    float delta1;               // 4
    float delta_1_m;            // 4
    float delta2;               // 4
    float delta2_m;             // 4
    float delta_a;              // 4
    float delta_s;              // 4
    float pwm_motor1;           // 4
    float pwm_motor2;           // 4
    // Add more fields here
    // string compassDirecton;
    char flight_id[8];          // 8 (null-terminated string, e.g. "FLIGHT1\0")
};
#pragma pack(pop)

#endif