#include "mbed.h"
#include "cli.h"
#include "EUSBSerial.h"
#include "ctrldRogallo.h"
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "Mutex_I2C.h"
#include "flight_packet.h"
#include "run_mode.h"
#include <iostream>
#include <cstdio>
#include <cstring>
#include "rtos.h"

// Instantiations
EUSBSerial pc;
Mutex_I2C i2c(PB_7, PB_8);
flash flash_chip(PA_7, PA_6, PA_5, PA_4, &pc);
uint32_t flash_addr = flash_chip.getNumPacketsWritten() * 256; 
ctrldRogallo ARES(&i2c);
CLI cli(&pc, &ARES, &flash_chip, &flash_addr);

// LEDs
DigitalOut led_B(PA_8);
DigitalOut led_G(PA_15);

// Initialize state struct
FlightPacket state; // potentially move this to a different scope

// Size in bytes of one flight packet 
const int FLIGHT_PACKET_SIZE =  sizeof(FlightPacket); 

// time step for PID controller to calculate derivative & integral
const float DT_CTRL          =  0.01; 


/** 
 *  @brief Autonomous flight mode. A PID controller is used to compute 
 *         an assymetric deflection command.
 *  @param ARES pointer to the ctrldRogallo flight object
 *  @param flash_addr pointer to the current system flash address 
 */
void autoFlight(ctrldRogallo* ARES, uint32_t* flash_addr){
    Timer execution_timer;
    execution_timer.start();
    auto EXECUTION_PERIOD = 20ms; 

    float theta_error;
    uint16_t packet_count;
    
    ThisThread::sleep_for(10ms);
    ARES->startAllSensorThreads(&pc); // REMOVE BEFORE FLIGHT: &pc
    ThisThread::sleep_for(10ms);
    ARES->startLogging(&flash_chip, &pc);
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
 *  @param flash_addr pointer to the current system flash address 
 */
void testMode(ctrldRogallo* ARES, uint32_t* flash_addr){
    Timer execution_timer;
    execution_timer.start();
    auto EXECUTION_PERIOD = 500ms; // 500ms = 2Hz

    char cmdBuf[32];
    float theta_error;
    uint16_t packet_count;
    
    pc.printf("entered test_mode...\n");
    ThisThread::sleep_for(100ms);
    ARES->startAllSensorThreads(&pc);
    pc.printf("started sensor threads...\n");
    ThisThread::sleep_for(100ms);
    pc.printf("entering startLogging...\n");
    ARES->startLogging(&flash_chip, &pc);
    pc.printf("started logging...\n\n");
    ThisThread::sleep_for(100ms);

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
        ARES->printCompactState(&pc);
        state = ARES->getState();

        // print number of packets logged (stored at the last two bytes of the flash chip) for debugging
        packet_count = flash_chip.getNumPacketsWritten();
        // pc.printf("Packets Logged: %d\n", packet_count);

        // State machine mode selection 
        switch (mode) {

            case FSM_IDLE: break; 

            case FSM_SEEKING: {
                // Ctrl Setup
                float target_heading = ARES->getTargetHeading();
                float heading_error = ARES->getHeadingError();
                float delta_a_cmd = ARES->computeCtrl(heading_error, DT_CTRL);

                // // TESTING
                // float delta_a_cmd = deflection;
                // if(up)  deflection += 0.1;
                // else    deflection -= 0.1;
                // if (deflection < -1 || deflection > 1) up = !up; 

                ARES->setLastFCcmd(delta_a_cmd);

                // MCPS Comms
                bool success = ARES->sendCtrl(delta_a_cmd);
                // pc.printf("CTRL SENT: %f; SUCCESS %i \n", delta_a_cmd, success); // debug
                break;
            }
            
            case FSM_GROUNDED: {
                ARES->stopLogging();
                ARES->killAllSensorThreads();
                break;
            }

            case FSM_SPIRAL: {
                float cmd = state.fc_cmd > 0 ? 1 : -1;
                ARES->setLastFCcmd(cmd);
                ARES->sendCtrl(cmd);
                break; 
            }

            // Unidentified Case
            default: break; 
        }

        // --- User Input Handling --- 
        if(pc.readline(cmdBuf, sizeof(cmdBuf))) {
            if(strcmp(cmdBuf, "quit") == 0) {
                pc.printf("\"quit\" cmd recieved...\n");
                ARES->stopLogging();
                ARES->killAllSensorThreads();
                ARES->setFSMMode(ModeFSM::FSM_GROUNDED);
            } else if (strcmp(cmdBuf, "seeking") == 0) {
                pc.printf("\"seeking\" cmd recieved...\n"); ARES->setFSMMode(ModeFSM::FSM_SEEKING);
            } else if (strcmp(cmdBuf, "idle") == 0) {
                pc.printf("\"idle\" cmd recieved...\n"); ARES->setFSMMode(ModeFSM::FSM_IDLE);
            } else if (strcmp(cmdBuf, "spiral") == 0) {
                pc.printf("\"spiral\" cmd recieved...\n"); ARES->setFSMMode(ModeFSM::FSM_SPIRAL);
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


/** 
 * @brief Determines what flight mode to enter used for readability 
 * @param mode the RunMode enum (Test or Flight)
 * @param ARES reference to the ctrldRogallo flight object
 */
void runMode(RunMode mode, ctrldRogallo* ARES) {
    ARES->setThreshold();
    ARES->updateFlightPacket();
    ThisThread::sleep_for(1s);
    switch (mode) {
        
        case RunMode::Test:  
            pc.printf("\nEntering Testing mode... %s\n", ' ');
            testMode(ARES, &flash_addr);                    // Testing mode
            pc.printf("==========================================================\n");
            break; 
        
        case RunMode::Flight:
            pc.printf("\nBeginning control sequence... %s\n", ' ');
            autoFlight(ARES, &flash_addr);                  // Flight Mode
            pc.printf("==========================================================\n");
            break; 
    }

    pc.printf("==============================%s\n", ' ');
    pc.printf("ARES data collection complete\n");
    pc.printf("==============================%s\n", ' ');
}


int main() {

    // Start the command-line interface in the background
    Thread cli_thread(osPriorityLow);
    cli_thread.start(callback(&cli, &CLI::run));

    // this either runs testMode or autoFlight loops
    runMode(RunMode::Test, &ARES);

    // Should never reach here
    while (true){
        ThisThread::sleep_for(1s);
    }
    
}
