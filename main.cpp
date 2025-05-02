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
    
    /*
     * Instantiates:    GPS gps(PA_2, PA_3), BMP280 bmp(PB_7, PB_8, 0xEE),
     *                  BNO055 bno(PB_7, PB_8, 0x51)
     * 
     * TODO:    should instantiate objects in main and pass pointers 
     *          OR just pass pins & stuff instead of hardcoding
     */
    ctrldRogallo ARES;
    EUSBSerial pc(true);
    flash fc(PA_7, PA_6, PA_5, PA_4, &pc);
    DigitalOut led_B(PA_8);
    led_B.write(0);

    // // FlightPacket packet;
    // int packetSize = sizeof(FlightPacket);
    // uint32_t eraseAddr = 0;
    // ThisThread::sleep_for(2s); // wait for serial port to connect?
    // pc.printf("\nStarting flash chip testing...");
    // // fc.eraseSector(currentFlashAddress); // to not overwrite stuff
    // uint32_t numPacketDump = 100;

    while (true){
        // // TESTING...
        // // THIS IS IN THE LOOP ON PURPOSE TO ONLY WRITE TO THE FIRST [packetSize] BYTES
        // uint32_t readAddress = 0;
        // uint32_t currentFlashAddress = 0; // start writing at address 0
        // pc.printf("\n\nErasing first sector of flash chip...");
        // fc.eraseSector(currentFlashAddress);
        // // get updated values from all sensors 
        // // and store them in the flight packet
        // pc.printf("\nUpdating flight packet...");
        ARES.updateFlightPacket();

        // get state and print values directly
        FlightPacket packet = ARES.getState();
        bool isGPSnan = false;
        if(packet.altitude_gps_m != packet.altitude_gps_m) isGPSnan =true; 
        pc.printf("\nPrinting State Direclty (No Flash Chip)");
        pc.printf("\n==================================");
        pc.printf("\nUTC\t\t:%f"
                    // "\n P, H, V DOP\t%f, %f, %f"
                    "\nPressure\t:%f Pa"
                    "\nTemperature\t:%f C"
                    "\nBMP Altitude\t:%f M"
                    "\nGPS Altitude\t:%f M"
                    "\nAltitude\t:%f M"
                    "\nYaw rate\t:%f rad/s"
                    "\nPitch rate\t:%f rad/s"
                    "\nRoll rate\t:%f rad/s", \
                    packet.timestamp_utc, packet.pressure_pa, packet.temp_c, packet.altitude_bmp_m, packet.altitude_gps_m, packet.altitude_m, packet.yaw_rate, packet.pitch_rate, packet.roll_rate);
                    pc.printf("\nCompass Direction\t: %s"
                    "\nHeading deg\t %f",packet.compassDirection, packet.headingTemp);
                    pc.printf("\nIs GPS alt Nan?\t: %d"
                    "\n is bmp alt nan?\t: %d", isnan(packet.altitude_gps_m),isGPSnan);
        pc.printf("\n==================================");
        ThisThread::sleep_for(100ms);


        // // write the packet to the flash chip
        // pc.printf("\nWriting packet to address: %d (Packet Size: %d)", currentFlashAddress, packetSize);
        // ARES.logDataTEST();
        // currentFlashAddress = fc.writePacket(currentFlashAddress, ARES.getState());
        // pc.printf("\nData written... New flash address: %d", currentFlashAddress);
        

        // // initialize struct and read one packet from flash chip
        // FlightPacket packet_read; // buffer
        // uint32_t nextAddress = fc.readPacket(readAddress, packet_read);

        // pc.printf("\nReading packet at address: %d", readAddress);
        // pc.printf("\n==================================\n");
        // fc.printCSVHeader();
        // fc.printPacketAsCSV(packet_read);
        // pc.printf("\n==================================");



        // // big write
        // pc.printf("\n\n Collecting 100 packets at 10Hz (~10s)...");
        // for (uint32_t i = 0; i < numPacketDump; i++) {
        //     ARES.updateFlightPacket();
        //     FlightPacket packet = ARES.getState();
        //     currentFlashAddress = fc.writePacket(currentFlashAddress, packet);
        //     ThisThread::sleep_for(100ms); // 10Hz
        // }

        // // big dumpy
        // pc.printf("\nDumping %d packets...", numPacketDump);
        // pc.printf("\n==================================\n");
        // fc.dumpAllPackets(numPacketDump);
        // pc.printf("\n==================================\n");
        // break;


        // // erase
        // pc.printf("\n Erasing sector at address: %d", eraseAddr);
        // fc.eraseSector(eraseAddr); // erase the sector

        // pc.printf("\nReading packet at address: %d", readAddress);
        // pc.printf("\n==================================");
        // pc.printf("\nFSM Mode\t:%d \nFix\t\t:%d \nLat Lon\t\t:%f, %f \nPressure\t:%f Pa \nTemperature\t:%f C", \
        //             packet_read.fsm_mode, packet_read.gps_fix, packet_read.latitude_deg, packet_read.longitude_deg, packet_read.pressure_pa, packet_read.temp_c);
        // pc.printf("\n==================================");
        ThisThread::sleep_for(5s);


    }

}


