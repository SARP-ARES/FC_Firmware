#include "mbed.h"
#include "EUSBSerial.h"
#include "ctrldRogallo.h"
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "Mutex_I2C.h"
#include "flight_packet.h"
#include <iostream>
#include <cstdio>
#include <cstring>
#include "rtos.h"

// Serial
EUSBSerial pc;

// ARES startup
Mutex_I2C i2c(PB_7, PB_8);
flash flash_chip(PA_7, PA_6, PA_5, PA_4, &pc);
ctrldRogallo ARES(&i2c, &flash_chip);

// LED
DigitalOut led_B(PA_8);
DigitalOut led_G(PA_15);
Thread thread;

FlightPacket state; // global state makes more sense than individual logging states 
                    // (could be moved to logging multi-function as a pointer)

enum FlightMode {
    test,
    flight
};

// = # of pages, 4.55 hours at 1Hz
// 16 pages per sector

const int FLIGHT_PACKET_SIZE =  sizeof(FlightPacket); // Size in bytes of one flight packet 
const int MAX_NUM_PACKETS    =  16384;

/* SET NUMBER OF PACKETS TO LOG || for for 1.5 hours of logging, log 5400 packets */
const int NUM_PACKETS_TO_LOG =  16000;
const float DT_CTRL          =  0.01; // time step for PID controller to calculate derivative & integral

const int packet_save_incr   = 100;

/** @brief clears all data off of the flash chip */
void clear_data() {
    led_B.write(1);
    Timer t;
    t.start();
    pc.printf("\nErasing flash chip memory...\n\n");
    int erase_err = flash_chip.eraseAll(); 
    ARES.resetPacketsLogged();
    int ms = t.read_ms(); 
    if (erase_err == 0) {
        pc.printf("...Erasing Complete... (%d ms)\n\n", ms);
    } else if(erase_err == 1) {
        pc.printf("...Erasing timed out... something went wrong");
    }
    led_B.write(0);
}

/** 
 *  @brief Autonomous flight mode. A PID controller is used to compute 
 *         an assymetric deflection command.
 *  @param ARES pointer to the ctrldRogallo flight object 
 */
void auto_flight(ctrldRogallo* ARES){
    Timer execution_timer;
    execution_timer.start();
    auto EXECUTION_PERIOD = 20ms; 

    float theta_error;
    uint16_t packet_count;
    
    ThisThread::sleep_for(10ms);
    ARES->startAllSensorThreads(&pc);
    ThisThread::sleep_for(10ms);
    ARES->startLogging(&pc);
    ThisThread::sleep_for(10ms);
    
    pc.printf("\n\n\t ARES ONLINE \n\n"); 

    // TESTING APPLICATIONS
    float deflection = 0;
    bool up = true;

    ModeFSM mode = ARES->getMode();

    while(mode != FSM_GROUNDED) {
        execution_timer.reset();

        // State handling
        ARES->updateFlightPacket();
        mode = ARES->getMode();
        state = ARES->getState();

        // State machine mode selection 
        switch (mode) {
            
            // Mode prior to apogee, waiting phase
            case FSM_IDLE: break; 

            // Mode for approaching the target radius
            case FSM_SEEKING: { 
                // Ctrl Setup
                float target_heading = ARES->getTargetHeading();
                float heading_error = ARES->getHeadingError();
                float delta_a_cmd = ARES->computeCtrl(heading_error, DT_CTRL);

                // State logging
                ARES->setLastFCcmd(delta_a_cmd);

                // Control Communication
                bool success = ARES->sendCtrl(delta_a_cmd);
                break;
            }
            
            // Mode once on the ground, ends flight logging
            case FSM_GROUNDED: {
                ARES->stopLogging();
                ARES->killAllSensorThreads();
                break;
            }

            // Mode once within target radius
            case FSM_SPIRAL: {
                float cmd = state.fc_cmd > 0 ? 1 : -1;
                ARES->setLastFCcmd(cmd);
                ARES->sendCtrl(cmd);
                break; 
            }

            // Unidentified Case
            default: break; 
        }

        // Event Scheduling, slow to 50hz if running faster than schedule 
        auto elapsed_time = execution_timer.elapsed_time();
        if (elapsed_time < EXECUTION_PERIOD) {
            ThisThread::sleep_for(
                chrono::duration_cast<Kernel::Clock::duration>(
                    EXECUTION_PERIOD - elapsed_time
                )
            );
        } // else run again asap

    }
}

/** 
 *  @brief Testing mode system prints & logs state until "quit" is entered into the CLI 
 *  @param ARES pointer to the ctrldRogallo flight object
 */
void test_mode(ctrldRogallo* ARES){
    Timer execution_timer;
    execution_timer.start();
    auto EXECUTION_PERIOD = 20ms; // 500ms = 2Hz

    char cmdBuf[32];
    float theta_error;
    uint16_t packet_count;

    ARES->startAllThreads(&pc);
    ARES->setThreshold();

    // TESTING APPLICATIONS
    float deflection = 0;
    bool up = true;

    ModeFSM mode = ARES->getMode();

    while(mode != FSM_GROUNDED) {
        execution_timer.reset();
        // Get current sensor data and put it in the state struct
        ARES->updateFlightPacket();
        mode = ARES->getMode();

        // print state for testing
        // ARES->printCompactState(&pc);
        state = ARES->getState();

        // State machine mode selection 
        switch (mode) {

            case FSM_IDLE: { break; }

            case FSM_SEEKING: {
                // Ctrl Setup
                // float target_heading = ARES->getTargetHeading();
                // float heading_error = ARES->getHeadingError();
                // float delta_a_cmd = ARES->computeCtrl(heading_error, DT_CTRL);

                // // TESTING
                // float delta_a_cmd = deflection;
                // if(up)  deflection += 0.1;
                // else    deflection -= 0.1;
                // if (deflection < -1 || deflection > 1) up = !up; 

                // ARES->setLastFCcmd(delta_a_cmd);

                // MCPS Comms
                // bool success = ARES->sendCtrl(delta_a_cmd);
                // pc.printf("CTRL SENT: %f; SUCCESS %i \n", delta_a_cmd, success); // debug
                break;
            }
            
            case FSM_GROUNDED: {
                ARES->stopAllThreads();
                break;
            }

            case FSM_SPIRAL: {
                // float cmd = state.fc_cmd > 0 ? 1 : -1;
                // ARES->setLastFCcmd(cmd);
                // ARES->sendCtrl(cmd);
                break; 
            }

            // Unidentified Case
            default: break; 
        }

        // pc.printf("PACKETS LOGGED %i\n", ARES->getPacketsLogged());

        // --- User Input Handling --- 
        if(pc.readline(cmdBuf, sizeof(cmdBuf))) {
            if(strcmp(cmdBuf, "quit") == 0) {
                pc.printf("\"quit\" cmd recieved...\n");
                ARES->stopAllThreads();
                ARES->setFSMMode(ModeFSM::FSM_GROUNDED);
            } else if (strcmp(cmdBuf, "seeking") == 0) {
                pc.printf("\"seeking\" cmd recieved...\n"); 
                ARES->setFSMMode(ModeFSM::FSM_SEEKING);
            } else if (strcmp(cmdBuf, "idle") == 0) {
                pc.printf("\"idle\" cmd recieved...\n"); 
                ARES->setFSMMode(ModeFSM::FSM_IDLE);
            } else if (strcmp(cmdBuf, "spiral") == 0) {
                pc.printf("\"spiral\" cmd recieved...\n"); 
                ARES->setFSMMode(ModeFSM::FSM_SPIRAL);
            }
        }

        // Event Scheduling, slow to 50hz if running faster than schedule 
        auto elapsed_time = execution_timer.elapsed_time();
        if (elapsed_time < EXECUTION_PERIOD) {
            ThisThread::sleep_for(
                chrono::duration_cast<Kernel::Clock::duration>(
                    EXECUTION_PERIOD - elapsed_time
                )
            );
        } // else run again asap
    }
}

/** @brief dumps all logged data in the serial port */
void dump_data(){
    uint16_t numPacketDump;
    flash_chip.read(0x3FFFFE, reinterpret_cast<uint8_t*> (&numPacketDump), 2);

    if (numPacketDump == 65535 || numPacketDump == 0) {
        numPacketDump = 0;
    }

    numPacketDump += packet_save_incr;
    
    // big dumpy
    pc.printf("\nDumping %d packets...", numPacketDump);
    pc.printf("\n==================================\n");
    flash_chip.dumpAllPackets(numPacketDump);
    pc.printf("==================================\n");

}

/** @brief verification step for the CLI */
void double_check() {
    char cmdBuf[32];
    pc.printf("Are you sure you want to erase all data from ARES?\n");
    pc.printf("1. \"yes\"\n2. \"no\"\n");
    
    Timer t;
    t.start();
    while (true) {
        if (pc.readline(cmdBuf, sizeof(cmdBuf))) {
            if (strcmp(cmdBuf, "yes") == 0 || strcmp(cmdBuf, "1") == 0) {
                pc.printf("\"yes\" received\n");
                ThisThread::sleep_for(1500ms);
                pc.printf("Ok... What's the password?\n");
                pc.printf("\n> ");
                ThisThread::sleep_for(4s);
                pc.printf("Just kidding :)\n");
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
            pc.printf("You took too long! Try again...");
            break;
        }
    }
}


/** @brief Sets the relative origin for ARES where ever we are currently */
void set_origin(ctrldRogallo* ARES) {
    // no lat/lon argument
    // sets current lat/lon as target/origin
    ARES->updateFlightPacket(); // get current lat/lon
    state = ARES->getState();

    if (state.latitude_deg != NAN && state.longitude_deg != NAN){
        ARES->setTarget(state.latitude_deg, state.longitude_deg);
        pc.printf("Origin has been set...\nLat, Lon: %f deg, %f deg\n", \
                state.latitude_deg, state.longitude_deg);
    } else { // lat/lon are nans... GPS doesn't have a fix yet.
        pc.printf("Origin has NOT been set because the GPS does not yet have fix.\n" \
                  "Make sure the antenna has a clear view of the sky and try again later.");
    }
}

/** 
 * @brief Sets the relative origin for ARES at the given coordinates
 * @param lat lattidue in deg 
 * @param lon longitude in deg
 */ 
void set_origin(ctrldRogallo* ARES, double lat, double lon) {
    // uses lat/lon arg to set target/origin
    ARES->setTarget(lat, lon);
    pc.printf("Origin has been set...\nLat, Lon: %lf deg, %lf deg\n", \
                lat, lon);
}

/** 
 * @brief Determines what flight (logging) mode to enter used for readability 
 * @param mode the FlightMode enum to enter
 * @param ARES reference to the ctrldRogallo flight object
 */
void flight_mode(FlightMode mode, ctrldRogallo* ARES) {
    ARES->setThreshold();
    ARES->updateFlightPacket();
    ThisThread::sleep_for(1s);
    switch (mode) {
        
        case test:  
            pc.printf("\nEntering Testing mode... %s\n", ' ');
            test_mode(ARES);                    // Testing mode
            pc.printf("==========================================================\n");
            break; 
        
        case flight:
            pc.printf("\nBeginning control sequence... %s\n", ' ');
            auto_flight(ARES);                  // Flight Mode
            pc.printf("==========================================================\n");
            break; 
    }

    pc.printf("==============================%s\n", ' ');
    pc.printf("ARES data collection complete\n");
    pc.printf("==============================%s\n", ' ');
}

void command_line_interface() {
    char cmd_buffer[32];  // user input buffer

    ThisThread::sleep_for(2s);

    bool cli_reset = true;
    while (true) {
        
        if (cli_reset) {
            pc.printf("\n\nARES is waiting for user input... What would you like to run?\n");
            pc.printf("0. \"flight_mode\"\n");
            pc.printf("1. \"test_mode\"\n");
            pc.printf("2. \"dump\"\n");
            pc.printf("3. \"set_origin\"\n");
            pc.printf("4. \"clear\"\n");
            pc.printf("5. \"dump_to_ser\"");
            pc.printf("\n> ");  // command prompt
            cli_reset = false;
        }

        memset(cmd_buffer, 0, sizeof(cmd_buffer)); // reset user input buffer

        // read user input
        if (pc.readline(cmd_buffer, sizeof(cmd_buffer))) {

            // flight log
            if (strcmp(cmd_buffer, "test_mode") == 0 || strcmp(cmd_buffer, "1") == 0) {
                pc.printf("\"test_mode\" cmd received.\nUse \"quit\" cmd to stop logging.\n" \
                "Use \"seeking\" cmd to force seeking FSM.\nUse \"idle\" to force idle\n" \
                "Use \"spiral\" cmd to force sprial FSM");
                ThisThread::sleep_for(1500ms);
                flight_mode(test, &ARES);
            }

            // flight log
            if (strcmp(cmd_buffer, "flight_mode") == 0 || strcmp(cmd_buffer, "6") == 0) {
                pc.printf("\"flight_mode\" cmd received.");
                ThisThread::sleep_for(1500ms);
                flight_mode(flight, &ARES);
            }

            // dump data
            else if (strcmp(cmd_buffer, "dump") == 0 || strcmp(cmd_buffer, "2") == 0) {
                pc.printf("\"dump\" cmd received\n");
                ThisThread::sleep_for(1500ms);
                dump_data();
            }

             // dump data
            else if (strcmp(cmd_buffer, "dump_to_ser") == 0 || strcmp(cmd_buffer, "5") == 0) {
                pc.printf("\"dump_to_ser\" cmd received\n");
                pc.printf("Waiting 30s, disconnect from serial port and start serial parser\n");
                ThisThread::sleep_for(30s);
                dump_data();
            }

            // set origin
            else if (strcmp(cmd_buffer, "set_origin") == 0 || strcmp(cmd_buffer, "3") == 0) {
                pc.printf("\"set_origin\" cmd received...\n");
                ThisThread::sleep_for(1500ms);
                pc.printf("Where would you like to set the origin?\n");
                // Wait for user input
                while (true) {
                    if (pc.readline(cmd_buffer, sizeof(cmd_buffer))) {
                        break;
                    }
                }
                double lat, lon;
                if (strcmp(cmd_buffer, "here") == 0) {
                    pc.printf("\"here\" received...\n");
                    ThisThread::sleep_for(1500ms);
                    set_origin(&ARES); // set current location as origin
                } else if (sscanf(cmd_buffer, "%lf, %lf", &lat, &lon) == 2) {
                    // get lat/lon from user input and set as origin
                    pc.printf("\"coordinates\" received...\n");
                    ThisThread::sleep_for(1500ms);
                    set_origin(&ARES, lat, lon);
                } else {
                    pc.printf("\nInvalid format. Use: \"<lat>, <lon>\" or type \"here\".\n");
                }
                
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

        ThisThread::sleep_for(100ms);
    }
}

int main() {
    command_line_interface();
}
