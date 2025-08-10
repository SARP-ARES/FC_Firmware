#include "mbed.h"
#include "EUSBSerial.h"
#include "ctrldRogallo.h"
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "flight_packet.h"
#include <iostream>
#include <cstring>



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
uint32_t numPackets = 16000;



void startup() {
    led_B.write(0);
    Timer t;
    t.start();

    pc.printf("\nErasing flash chip memory...\n\n");
    fc.eraseAll(); 
    int ms = t.read_ms();   
    pc.printf("...Erasing Complete... (%d ms)\n\n", ms);
    
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

    // --------------------------------------------------------

    pc.printf("\nCollecting %d %d-byte packets at 1Hz...\n", numPacketLog, packetSize);
    pc.printf("\nARES IS READY TO INSTALL\n");
    led_G.write(1);


    // big write
    for (int i = 0; i < numPacketLog; i++) {

        ARES.updateFlightPacket(); 
   
        FlightPacket state = ARES.getState(); // extract state variables
        currentFlashAddress = fc.writePacket(currentFlashAddress, state); // write state variables to flash chip
        pc.printf("Apogee Counter %f\n", state.apogee_counter);

        if (state.fsm_mode == FSM_SEEKING) { // mode is set after apogee detection
            ctrl_trigger.write(0); // signal to control sequence on MCPS 
        }
    }

    pc.printf("=================================%s\n", ' ');
    pc.printf(" ARES DATA COLLECTION FINISHED %s\n", ' ');
    pc.printf("=================================%s\n", ' ');

} 

void flight_log(){

    ctrldRogallo ARES; 

    char cmdBuf[32];

    ThisThread::sleep_for(5s);

    ARES.updateFlightPacket();
    ARES.setThreshold(); 

    pc.printf("Beginning data collection... %s\n", ' ');

    DigitalOut ctrl_trigger(PB_3); 
    ctrl_trigger.write(1); 
    
    ThisThread::sleep_for(1s);
    uint32_t currentFlashAddress = 0;

    FlightPacket state;

    while(state.fsm_mode != FSM_GROUNDED){

        ARES.updateFlightPacket();
        state = ARES.getState();
        pc.printf("Apogee Counter %d\n", state.apogee_counter);
        pc.printf("Altitude %f\n", state.altitude_m);
        pc.printf("Apogee Detected %d\n", state.apogee_detected);
        pc.printf("FSM mode %d\n", state.fsm_mode);
        pc.printf("Grounded Counter: %d \n", state.groundedCounter);

        currentFlashAddress = fc.writePacket(currentFlashAddress, state);
        if (state.fsm_mode == FSM_SEEKING) { // mode is set after apogee detection
            ctrl_trigger.write(0); // signal to control sequence on MCPS 
        }

        // break on "quit" command
        if(pc.readline(cmdBuf, sizeof(cmdBuf))) {
            if(strcmp(cmdBuf, "quit") == 0) {
                break;
            }
        }
    }

    pc.printf("=================================%s\n", ' ');
    pc.printf(" ARES DATA COLLECTION FINISHED %s\n", ' ');
    pc.printf("=================================%s\n", ' ');
}



void dump(){

    uint32_t numPacketDump;
    fc.read(0x3FFFFE, reinterpret_cast<uint8_t*> (&numPacketDump), 2);

    // big dumpy
    pc.printf("\nDumping %d packets...", numPacketDump);
    pc.printf("\n==================================\n");
    fc.dumpAllPackets(numPacketDump);
    pc.printf("==================================\n");
    
}


void command_line_interface(){

    ThisThread::sleep_for(2s);
    pc.printf("\nWaiting for console input...\n\n");
    char cmd_buffer[32];

    while(true) {

        if(pc.readline(cmd_buffer, sizeof(cmd_buffer))) {

           if (strcmp(cmd_buffer, "flightlog") == 0){
               pc.printf("Running Flight Log\n\n");
                flight_log();
                command_line_interface();
            } else if(strcmp(cmd_buffer,"startup") == 0) {
                pc.printf("Running Startup\n");
                startup();
                command_line_interface();
            } else if(strcmp(cmd_buffer,"dump") == 0) {
                pc.printf("Dumping Packets\n");
                dump();
                command_line_interface();
            } 

            break; 
        }

        ThisThread::sleep_for(100ms);
    }
}



int main() {
    command_line_interface();
    /*
     * PROCEDURE
     * 1) startup()     - erases all chip memory
     * 2) flight_log()  - logs data during flight
     * 3) dump()        - prints all data on flash chip as a CSV
     */
}

