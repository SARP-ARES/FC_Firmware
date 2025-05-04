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
DigitalOut led_B(PA_8);
DigitalOut led_G(PA_15);
Thread thread;

// = # of pages, 4.55 hours at 1Hz
// 16 pages per sector
uint32_t packetSize = sizeof(FlightPacket);
uint32_t MAX_NUM_PACKETS = 16384; 
uint32_t eraseAddr = 0;


/*
 * SET NUMBER OF PACKETS TO LOG
 * for for 1.5 hours of logging, log 5400 packets
 */
uint32_t numPackets = 5400; 


void startup() {
    led_B.write(0);
    pc.printf("\nErasing flash chip memory...\n\n");
    fc.eraseAll();    
    pc.printf("...Erasing Complete...\n\n");
    ThisThread::sleep_for(1s);
    led_B.write(1);
    pc.printf("ARES IS READY TO BEGIN FLIGHT LOG\n");
}



void flight_log(uint32_t numPacketLog) {
    ctrldRogallo ARES; 

    /* 
     * start ctrl_trigger high, write low once apogee is detected and fsm_mode =
     * to trigger control sequence
     */
    DigitalOut ctrl_trigger(PB_3); 
    ctrl_trigger.write(1); 
    
    ThisThread::sleep_for(1s);
    uint32_t currentFlashAddress = 0;

    pc.printf("\nCollecting %d %d-byte packets at 1Hz...\n", numPacketLog, packetSize);
    pc.printf("\nARES IS READY TO INSTALL\n");
    led_G.write(1);

    // big write
    for (uint32_t i = 0; i < numPacketLog; i++) {
        ARES.resetFlightPacket();   // set all state variables to NAN or equivalent
        ARES.updateFlightPacket();  // update all state variables with sensor data
        FlightPacket state = ARES.getState();   // extract state variables
        currentFlashAddress = fc.writePacket(currentFlashAddress, state);   // write state variables to flash chip

        if (state.fsm_mode == FSM_SEEKING) { // mode is set after apogee detection
            ctrl_trigger.write(0); // signal to control sequence on MCPS 
        }
    }
}


void dump(uint32_t numPacketDump){
    // big dumpy
    pc.printf("\nDumping %d packets...", numPacketDump);
    pc.printf("\n==================================\n");
    fc.dumpAllPackets(numPacketDump);
    pc.printf("==================================\n");
}


void ledFlashy30() {
    Timer t;
    t.start();

    while (t.read_ms() < 30*1000) {
        led_B.write(1);
        led_G.write(0);
        ThisThread::sleep_for(100ms);
        led_B.write(0);
        led_G.write(1);
        ThisThread::sleep_for(100ms);
    }
    led_G.write(0);
}


int main() {
    ThisThread::sleep_for(1s); // wait for serial port to connect
    pc.printf("\n30s to flash before main program begins..\n");
    thread.start(ledFlashy30);
    ThisThread::sleep_for(30s);
    pc.printf("\nEntering main program...\n");

    /*
     * PROCEDURE
     * 1) startup()     - erases all chip memory
     * 2) flight_log()  - logs data during flight
     * 3) dump()        - prints all data on flash chip as a CSV
     */

    // startup();
    flight_log(numPackets);
    // dump(numPackets);

}
