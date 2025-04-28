// Testing data logging with flash chip
// #include "ThisThread.h"
#include "mbed.h"
#include "EUSBSerial.h"
#include "ctrldRogallo.h"
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "flight_packet.h"
// #include <cstdint>


int main() {
    
    // instantiate flash chip
    // Instantiates: GPS gps(PA_2, PA_3), BMP280 bmp(PB_7, PB_8, 0xEE), BNO055 bno(PB_7, PB_8, 0x51), fc(PA_7, PA_6, PA_5, PA_4)
    // Q: should make objects in main and pass pointers instead?
    ctrldRogallo ARES;
    EUSBSerial pc(true);
    DigitalOut led_B(PA_8);
    led_B.write(0);

    // FlightPacket packet;
    int packetSize = sizeof(FlightPacket);
    uint32_t readAddress = 0;
    ThisThread::sleep_for(2s); // wait for serial port to connect?
    pc.printf("\nStarting flash chip testing...");
    while (true){
        // TESTING...
        // THIS IS IN THE LOOP ON PURPOSE TO ONLY WRITE TO THE FIRST [packetSize] BYTES
        pc.printf("\n\nErasing first sector of flash chip...");
        uint32_t flashAddress = 0; // start writing at address 0
        ARES.fc.eraseSector(flashAddress); // to not overwrite stuff
        
        // get updated values from all sensors 
        // and store them in the flight packet
        pc.printf("\nUpdating flight packet...");
        ARES.updateFlightPacket();

        // write the packet to the flash chip
        pc.printf("\nWriting packet to address: %d (Packet Size: %d)", ARES.currentFlashAddress, packetSize);
        ARES.logDataTEST();
        pc.printf("\nData written... New flash address: %d", ARES.currentFlashAddress);
        

        // initialize struct and read one packet from flash chip
        FlightPacket packet_read; // buffer
        ARES.fc.readPacket(readAddress, packet_read);

        pc.printf("\nReading packet at address: %d", readAddress);
        pc.printf("\n==================================");
        pc.printf("\nFSM Mode\t:%d \nFix\t\t:%d \nLat\t\t:%f \nLon\t\t:%f \nPressure\t:%f Pa \nTemperature\t:%f", \
                    packet_read.fsm_mode, packet_read.gps_fix, packet_read.latitude_deg, packet_read.longitude_deg, packet_read.pressure_pa, packet_read.temp_c);
        pc.printf("\n==================================");
        ThisThread::sleep_for(5s);


    }

}
