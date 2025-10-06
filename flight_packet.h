#include "mbed.h"
#ifndef FLIGHT_PACKET_H
#define FLIGHT_PACKET_H

#pragma pack(push, 1) // remove padding to ensure correct memory layout
struct FlightPacket {
    float timestamp_utc;         // 4 bytes
    uint8_t fsm_mode;            // 1
    uint16_t gps_fix;            // 2
    uint8_t gps_antenna_status;  // 1
    float heading_deg;           // 4
    float target_heading_deg;    // 4
    float heading_error_deg;     // 4
    float h_speed_m_s;           // 4
    float v_speed_m_s;           // 4
    double latitude_deg;         // 8
    double longitude_deg;        // 8
    float altitude_gps_m;        // 4
    float altitude_bmp_m;        // 4
    float altitude_m;            // 4
    float pos_east_m;            // 4
    float pos_north_m;           // 4
    float distance_to_target_m;  // 4
    float temp_c;                // 4
    float pressure_pa;           // 4
    float delta_1_deg;           // 4
    float delta_1_m;             // 4
    float delta_2_deg;           // 4
    float delta_2_m;             // 4
    float delta_a;               // 4
    float delta_s;               // 4
    float pwm_motor1;            // 4
    float pwm_motor2;            // 4
    float fc_cmd;                // 4
    uint32_t apogee_counter;     // 4
    uint8_t apogee_detected;     // 1


    // --- BNO055 Sensor Fields ---

    // Gyroscope
    float yaw_rate;                 
    float pitch_rate;           
    float roll_rate;           

    // Accelerometer
    float bno_acc_x;
    float bno_acc_y;
    float bno_acc_z;

    // Magnetometer
    float bno_mag_x;
    float bno_mag_y;
    float bno_mag_z;

    // Euler Angles
    float bno_eul_x;
    float bno_eul_y;
    float bno_eul_z;

    // Linear Acceleration
    float bno_lin_x;
    float bno_lin_y;
    float bno_lin_z;

    // Gravity Vector
    float bno_grav_x;
    float bno_grav_y;
    float bno_grav_z;

    // Quaternion (if available)
    float bno_quat_w;
    float bno_quat_x;
    float bno_quat_y;
    float bno_quat_z;

    char compass_heading[3];         // 3
    uint32_t groundedCounter;
    float prevAlt; 

    // --- END BNO055 Sensor Fields ---

    char flight_id[8];          // 8 (null-terminated string
};
#pragma pack(pop)

#endif
