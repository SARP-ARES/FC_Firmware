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


ctrldRogallo ARES;
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



void clear_data() {
    led_B.write(0);
    Timer t;
    t.start();

    pc.printf("\nErasing flash chip memory...\n\n");
    fc.eraseAll(); 
    int ms = t.read_ms();   
    pc.printf("...Erasing Complete... (%d ms)\n\n", ms);
    
    led_B.write(1);
}



void flight_log(ctrldRogallo* ARES, uint32_t numPacketLog) {

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

        ARES->updateFlightPacket(); 
   
        FlightPacket state = ARES->getState(); // extract state variables
        currentFlashAddress = fc.writePacket(currentFlashAddress, state); // write state variables to flash chip

        if (state.fsm_mode == FSM_SEEKING) { // mode is set after apogee detection
            ctrl_trigger.write(0); // signal to control sequence on MCPS 
        }
    }

    pc.printf("=================================%s\n", ' ');
    pc.printf(" ARES DATA COLLECTION FINISHED %s\n", ' ');
    pc.printf("=================================%s\n", ' ');

} 

void flight_log(ctrldRogallo* ARES){

    char cmdBuf[32];

    ARES->updateFlightPacket();
    ARES->setThreshold(); 

    pc.printf("Beginning data collection... %s\n", ' ');

    DigitalOut ctrl_trigger(PB_3); 
    ctrl_trigger.write(1); 
    
    ThisThread::sleep_for(1s);
    uint32_t currentFlashAddress = 0; // start writing at the first flash address

    FlightPacket state;

    while(state.fsm_mode != FSM_GROUNDED){

        ARES->updateFlightPacket();
        state = ARES->getState();
        
        // pc.printf("Lat, Lon, Alt: \t %f, %f, %f\n", state.latitude_deg, state.longitude_deg, state.altitude_m);
        // pc.printf("FSM mode %d\n", state.fsm_mode);
        // pc.printf("Apogee Counter %d\n", state.apogee_counter);
        // pc.printf("Apogee Detected %d\n", state.apogee_detected);
        // pc.printf("Grounded Counter: %d \n", state.groundedCounter);

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

    pc.printf("==============================%s\n", ' ');
    pc.printf("\"quit\" cmd recieved...\n");
    pc.printf("ARES data collection complete\n");
    pc.printf("==============================%s\n", ' ');
}



void dump_data(){

    uint16_t numPacketDump;
    fc.read(0x3FFFFE, reinterpret_cast<uint8_t*> (&numPacketDump), 2);

    if (numPacketDump == 0xFFFF || numPacketDump == 0) {
        // If it's the default erased value, there are no packets to dump
        pc.printf("There are no packets to dump!");
        ThisThread::sleep_for(1500ms);

    } else {
        // big dumpy
        pc.printf("\nDumping %d packets...", numPacketDump);
        pc.printf("\n==================================\n");
        fc.dumpAllPackets(numPacketDump);
        pc.printf("==================================\n");
    }

}


void double_check() {
    char cmdBuf[32];
    pc.printf("Are you sure you want to erase all data from ARES?\n");
    pc.printf("1. \"yes\"\n2. \"no\"\n3. \"yo mama\"\n");
    
    Timer t;
    t.start();
    while (true) {
        if (pc.readline(cmdBuf, sizeof(cmdBuf))) {
            if (strcmp(cmdBuf, "yes") == 0 || strcmp(cmdBuf, "1") == 0) {
                pc.printf("\"yes\" received\n");
                ThisThread::sleep_for(1500ms);
                pc.printf("Ok... What's the password?\n");
                ThisThread::sleep_for(4s);
                pc.printf("Just kidding :)");
                ThisThread::sleep_for(1s);
                clear_data();
                break;
            } else if (strcmp(cmdBuf, "no") == 0 || strcmp(cmdBuf, "2") == 0) {
                pc.printf("\"no\" received\n");
                ThisThread::sleep_for(1500ms);
                break;
            } else if (strcmp(cmdBuf, "yo mama") == 0 || strcmp(cmdBuf, "3") == 0) {
                pc.printf("\"yo mama\" received\n");
                ThisThread::sleep_for(1500ms);
                pc.printf("That wasn't very nice :(");
                ThisThread::sleep_for(1s);
                break;
            }
        }
        // break (don't erase) if there is no response in 15 seconds
        if (t.read_ms() > 15000) {
            pc.printf("You took too damn long! Try again...");
            break;
        }
    }
        
}


void set_origin(ctrldRogallo* ARES) {
    ARES->updateFlightPacket(); // get current coordinates
    ARES->gps.setOriginECEFr(); // set coordinates as origin for LTP
    FlightPacket state = ARES->getState();
    posECEFr origin = ARES->gps.getOriginECEFr();
    pc.printf("LTP origin has been set...\nLat, Lon, Alt: %f deg, %f deg, %.3f m\nECEFr (x, y, z): %.3f m, %.3f m, %.3f m", \
    state.latitude_deg, state.longitude_deg, state.altitude_m, origin.x, origin.y, origin.z);
}


void command_line_interface() {
    char cmd_buffer[32];  // user input buffer

    ThisThread::sleep_for(2s);

    bool cli_reset = true;
    while (true) {
        
        if (cli_reset) {
            pc.printf("\n\nARES is waiting for user input... What would you like to run?\n");
            pc.printf("1. \"flight_log\"\n");
            pc.printf("2. \"dump\"\n");
            pc.printf("3. \"set_origin\"\n");
            pc.printf("4. \"clear\"\n");
            pc.printf("\n> ");  // command prompt
            cli_reset = false;
        }

        memset(cmd_buffer, 0, sizeof(cmd_buffer)); // reset user input buffer

        // read user input
        if (pc.readline(cmd_buffer, sizeof(cmd_buffer))) {

            // flight log
            if (strcmp(cmd_buffer, "flight_log") == 0 || strcmp(cmd_buffer, "1") == 0) {
                pc.printf("\"flight_log\" cmd received. Use \"quit\" cmd to stop logging.\n");
                ThisThread::sleep_for(1500ms);
                flight_log(&ARES);
            }

            // dump data
            else if (strcmp(cmd_buffer, "dump") == 0 || strcmp(cmd_buffer, "2") == 0) {
                pc.printf("\"dump\" cmd received\n");
                ThisThread::sleep_for(1500ms);
                dump_data();
            }

            // set origin
            else if (strcmp(cmd_buffer, "set_origin") == 0 || strcmp(cmd_buffer, "3") == 0) {
                pc.printf("\"set_origin\" cmd received\n");
                ThisThread::sleep_for(1500ms);
                set_origin(&ARES);
            }

            // clear data
            else if (strcmp(cmd_buffer, "clear") == 0 || strcmp(cmd_buffer, "4") == 0) {
                pc.printf("\"clear\" cmd received\n");
                ThisThread::sleep_for(1500ms);
                double_check(); // make sure the user wants to clear the data
            }

            // hello
            else if (strcmp(cmd_buffer, "hello") == 0) {
                pc.printf("\"hello\" received\n");
                ThisThread::sleep_for(1500ms);
                pc.printf("\nhello! :)\n");
                ThisThread::sleep_for(1500ms);
            }

            // Unknown command
            else {
                pc.printf("Unknown command: %s\n", cmd_buffer);
            }

            cli_reset = true; // reset CLI after user input is recieved 
        }

        // Avoid hammering CPU
        ThisThread::sleep_for(100ms);
    }
}


int main() {
    command_line_interface();
}

