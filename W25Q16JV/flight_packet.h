#include "mbed.h"
#ifndef FLIGHT_PACKET_H
#define FLIGHT_PACKET_H

#pragma pack(push, 1) // remove padding to ensure correct memory layout
struct FlightPacket {
    uint32_t timestamp_utc;     // 4 bytes
    uint8_t fsm_mode;           // 1
    uint8_t gps_fix;            // 1
    float heading_deg;          // 4
    float target_heading_deg;   // 4
    float groundspeed_m_s;      // 4
    float v_speed_m_s;          // 4
    float latitude_deg;         // 4
    float longitude_deg;        // 4
    float altitude_m;           // 4
    float pos_east_m;           // 4
    float pos_north_m;          // 4
    float pos_up_m;             // 4
    float temp_c;               // 4
    float pressure_pa;          // 4
    float delta1;               // 4
    float delta_1_m;            // 4
    float delta2;               // 4
    float delta2_m;             // 4
    float delta_a;              // 4
    float delta_s;              // 4
    float pwm_motor1;           // 4
    float pwm_motor2;           // 4
    float fc_cmd;               // 4
    char id[8];                 // 8 (null-terminated string, e.g. "FLIGHT1\0")
    // Add more fields as needed
};
#pragma pack(pop)

#endif