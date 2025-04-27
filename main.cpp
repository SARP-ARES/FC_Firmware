// Testing data logging with flash chip
#include "ThisThread.h"
#include "mbed.h"
#include "EUSBSerial.h"
#include "flash.h"
#include "flight_packet.h"
#include <cstdint>

int main() {
    
    // instantiate flash chip
    flash fc(PA_7, PA_6, PA_5, PA_4); // MOSI, MISO, SCLK, CS (Flight Computer)

    EUSBSerial pc(true);
    DigitalOut led_B(PA_8);
    led_B.write(0);

    pc.printf("CHECK");
    // make test data
    float lat = 90.0;
    float lon = 180.0;
    // int pSize_float_bytes = 8; // 2 floats = 8 bytes

    FlightPacket packet;
    int packetSize = sizeof(FlightPacket);
    uint32_t readAddress = 0;
    // uint32_t flashAddress = 0; // start writing at address 0
    pc.printf("Starting flash chip testing...");
    while (true){
        // TESTING...
        // THIS IS IN THE LOOP ON PURPOSE TO ONLY WRITE TO THE FIRST [packetSize] BYTES
        uint32_t flashAddress = 0; // start writing at address 0
        fc.eraseSector(flashAddress); // to not overwrite stuff


        /* FLIGHT PACKET STRUCT FOR REFERENCE
        struct FlightPacket {
        uint32_t timestamp_utc;     // 4 bytes
        uint8_t fsm_mode;           // 1
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
        char id[8];                 // 8 (
        */

        // store data in the packet struct
        packet.timestamp_utc = 0;
        packet.fc_cmd = 0.8;
        packet.heading_deg = 100;
        packet.latitude_deg = 47.0;
        packet.longitude_deg = -122.0;
        packet.fsm_mode = 3;
        // packet.pressure_pa = NAN;

        // write the packet to the flash chip
        flashAddress = fc.writePacket(flashAddress, packet);

        // initialize struct and read one packet from flash chip
        FlightPacket packet_read; // buffer
        fc.readPacket(readAddress, packet_read);

        pc.printf("\n==================================");
        pc.printf("\nFSM Mode:\t%d \nLat:\t\t%f \nLon:\t\t%f \nPressure:\t%f", \
                    packet_read.fsm_mode, packet_read.latitude_deg, packet_read.longitude_deg, packet.pressure_pa);
        ThisThread::sleep_for(2s);

    }


}

