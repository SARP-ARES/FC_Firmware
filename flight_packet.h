#include "mbed.h"
#include <string>
#ifndef FLIGHT_PACKET_H
#define FLIGHT_PACKET_H

#pragma pack(push, 1) // remove padding to ensure correct memory layout
struct FlightPacket {
    float timestamp_utc;        // 4 bytes
    uint8_t fsm_mode;           // 1
    uint16_t gps_fix;           // 2
    float heading_deg;          // 4
    float target_heading_deg;   // 4
    float h_speed_m_s;          // 4
    float v_speed_m_s;          // 4
    float latitude_deg;         // 4
    float longitude_deg;        // 4
    float altitude_gps_m;       // 4
    float altitude_bmp_m;       // 4
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
    bool apogee_detected;       // 1
    float yaw_rate;             // 4
    float pitch_rate;           // 4 
    float roll_rate;            // 4
    // Add more fields here
    float headingTemp; 
    string compassDirection;
    
    char flight_id[8];          // 8 (null-terminated string, e.g. "FLIGHT1\0")

    
};
#pragma pack(pop)

#endif