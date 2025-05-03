#include "mbed.h"
#include "EUSBSerial.h"
#include "ctrldRogallo.h"
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "flight_packet.h"


EUSBSerial pc;
flash fc(PA_7, PA_6, PA_5, PA_4, &pc);

// = # of pages, 4.55 hours at 1Hz
// 16 pages per sector
uint32_t packetSize = sizeof(FlightPacket);
uint32_t MAX_NUM_PACKETS = 16384; 
uint32_t eraseAddr = 0;


/*
 * SET NUMBER OF PACKETS TO LOG
 */
uint32_t numPacketDump = 30;


void reset(){
    DigitalOut led_B(PA_8);
    led_B.write(0);
    // flash fc(PA_7, PA_6, PA_5, PA_4, &pc);
    // EUSBSerial pc;
    pc.printf("\nErasing flash chip memory...\n\n");
    fc.eraseAll();    
    pc.printf("...Erasing Complete...\n\n");
    ThisThread::sleep_for(1s);
    led_B.write(1);
    pc.printf("ARES READY FOR FLIGHT");
}




void dump(){
    // big dumpy
    pc.printf("\nDumping %d packets...", numPacketDump);
    pc.printf("\n==================================\n");
    fc.dumpAllPackets(numPacketDump);
    pc.printf("==================================\n");
}


/*
 * Instantiates:    GPS gps(PA_2, PA_3), BMP280 bmp(PB_7, PB_8, 0xEE),
 *                  BNO055 bno(PB_7, PB_8, 0x51)
 * 
 * TODO:    should instantiate objects in main and pass pointers 
 *          OR just pass pins & stuff instead of hardcoding
 */
int main() {
    ThisThread::sleep_for(1s);
    pc.printf("Entering main program...");


    // reset();

    fc.eraseSector(0);

    // flash fc(PA_7, PA_6, PA_5, PA_4, &pc);
    ctrldRogallo ARES;
    

    // start high, write low once apogee is detected
    // to trigger control sequence
    DigitalOut ctrl_trigger(PB_3); 
    ctrl_trigger.write(1); 

    Timer t;
    

    ThisThread::sleep_for(1s);
    uint32_t currentFlashAddress = 0;

    pc.printf("\n\nCollecting %d %d-byte packets at 1Hz...", numPacketDump, packetSize);
    
    // big write
    for (uint32_t i = 0; i < numPacketDump; i++) {
        ARES.resetFlightPacket();
        ARES.updateFlightPacket();
        FlightPacket packet = ARES.getState();
        currentFlashAddress = fc.writePacket(currentFlashAddress, packet);
        ThisThread::sleep_for(100ms); 
    }

    dump();
    
}
