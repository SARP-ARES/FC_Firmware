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
    DigitalOut ctrl_trigger(PB_3); 
    ctrl_trigger.write(1); 
    // start high, write low once apogee is detected
    // to trigger control sequence

    led_B.write(1);
    Timer t;

    // FlightPacket packet;
    uint32_t packetSize = sizeof(FlightPacket);
    uint32_t MAX_NUM_PACKETS = 16384; // = # of pages, 4.55 hours at 1Hz
    // 16 pages per sector
    uint32_t eraseAddr = 0;
    ThisThread::sleep_for(2s); // wait for serial port to connect?
    // pc.printf("\nStarting flash chip testing...");
    // fc.eraseSector(currentFlashAddress); // to not overwrite stuff

    uint32_t numPacketDump = 30;

    // pc.printf("\nErasing flash chip memory...\n\n");
    // fc.eraseAll();    
    // pc.printf("...Erasing Complete...\n\n");
    // ThisThread::sleep_for(1s);
    // pc.printf("ARES READY FOR FLIGHT")

    while (true){
        // uint32_t readAddress = 0;
        uint32_t currentFlashAddress = 0; // start writing at address 0
        pc.printf("\n\nErasing first sector of flash chip...");
        fc.eraseSector(currentFlashAddress);


        // get updated values from all sensors 
        // and store them in the flight packet
        // pc.printf("\nUpdating flight packet...");

        // ARES.resetFlightPacket(); // Set everything to nans
        // ARES.updateFlightPacket();

        // // get state and print values directly
        // FlightPacket packet = ARES.getState();
        // pc.printf("\nPrinting State Direclty (No Flash Chip)");
        // pc.printf("\n================================================");
        // pc.printf("\nUTC\t\t\t:%f"
        //             "\nFSM Mode\t\t:%d"
        //             "\nFix\t\t\t:%d"
        //             "\nLat, Lon, Alt\t\t:%f, %f, %f"
        //             "\ne, n, u\t\t\t:%f, %f, %f"
        //             "\nBMP alt, GPS alt\t:%f, %f"
        //             // "\n P, H, V DOP\t%f, %f, %f"
        //             "\nPressure\t\t:%f Pa"
        //             "\nTemperature\t\t:%f C"
        //             "\nyaw, pitch, roll rates\t:%f, %f, %f"
        //             "\nApogee Counter\t\t:%d"
        //             "\nAPOGEE DETECTED\t\t:%d",
        //             packet.timestamp_utc,
        //             packet.fsm_mode,
        //             packet.gps_fix,
        //             packet.latitude_deg, packet.longitude_deg, packet.altitude_m,
        //             packet.pos_east_m, packet.pos_north_m, packet.pos_up_m,
        //             packet.altitude_bmp_m,
        //             packet.altitude_gps_m,
        //             packet.pressure_pa,
        //             packet.temp_c,
        //             packet.yaw_rate, packet.pitch_rate, packet.roll_rate,
        //             packet.apogee_counter,
        //             packet.apogee_detected);
        // pc.printf("\n================================================");



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


        // big write
        pc.printf("\n\nCollecting %d %d-byte packets at 1Hz...", numPacketDump, packetSize);
        // t.start();
        // uint32_t count_packets = 0;
        // while (count_packets < numPacketDump) { // break at 10 seconds
        //     if (t.read_ms() >= 100) { // update and log at 10Hz
        //         ARES.resetFlightPacket(); // initialize state struct
        //         ARES.updateFlightPacket(); // update state
        //         FlightPacket packet = ARES.getState(); // extract state into packet
        //         currentFlashAddress = fc.writePacket(currentFlashAddress, packet);
        //         count_packets++;
        //     }
        // }
        

        for (uint32_t i = 0; i < numPacketDump; i++) {
            ARES.resetFlightPacket();
            ARES.updateFlightPacket();
            FlightPacket packet = ARES.getState();
            currentFlashAddress = fc.writePacket(currentFlashAddress, packet);
            ThisThread::sleep_for(100ms); 
        }

        // big dumpy
        pc.printf("\nDumping %d packets...", numPacketDump);
        pc.printf("\n==================================\n");
        fc.dumpAllPackets(numPacketDump);
        pc.printf("\n==================================\n");
        break;


        // // erase
        // pc.printf("\n Erasing sector at address: %d", eraseAddr);
        // fc.eraseSector(eraseAddr); // erase the sector

        // pc.printf("\nReading packet at address: %d", readAddress);
        // pc.printf("\n==================================");
        // pc.printf("\nFSM Mode\t:%d \nFix\t\t:%d \nLat Lon\t\t:%f, %f \nPressure\t:%f Pa \nTemperature\t:%f C", \
        //             packet_read.fsm_mode, packet_read.gps_fix, packet_read.latitude_deg, packet_read.longitude_deg, packet_read.pressure_pa, packet_read.temp_c);
        // pc.printf("\n==================================");
        // ThisThread::sleep_for(100ms);


    }

}
