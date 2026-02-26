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
#include <iostream>
#include <cstdio>
#include <cstring>
#include "rtos.h"

// Instantiations
EUSBSerial pc;
Mutex_I2C i2c(PB_7, PB_8);
flash flash_chip(PA_7, PA_6, PA_5, PA_4, &pc);
uint32_t flash_addr = flash_chip.getNumPacketsWritten() * 256; 
ctrldRogallo ARES(&i2c, &flash_chip);
CLI cli(&pc, &ARES, &flash_chip, &flash_addr);

// Initialize state struct
FlightPacket state; // potentially move this to a different scope

// Main loop execution speed
const Kernel::Clock::duration MAIN_EXECUTION_PERIOD = 50ms; 

// time step for PID controller to calculate derivative & integral
// cast main loop execution period to seconds
const float DT_CTRL          =  std::chrono::duration_cast<std::chrono::seconds>(MAIN_EXECUTION_PERIOD).count();

// Size in bytes of one flight packet 
const int FLIGHT_PACKET_SIZE =  sizeof(FlightPacket); 


void executeFlightLogic() {
    // State handling
    ARES.updateFlightPacket();
    ModeFSM mode = ARES.getMode();
    state = ARES.getState();

    // State machine mode selection 
    switch (mode) {
        
        // Mode prior to apogee, waiting phase
        case FSM_IDLE: {
            break; 
        }

        // Mode for approaching the target radius
        case FSM_SEEKING: { 
            // Ctrl Setup
            float target_heading = ARES.getTargetHeading();
            float heading_error = ARES.getHeadingError();
            float delta_a_cmd = ARES.computeCtrl(heading_error, DT_CTRL);

            // set lat command for spiral logic
            ARES.setLastFCcmd(delta_a_cmd);

            // Send control command to MC/PC
            bool success = ARES.sendCtrl(delta_a_cmd);
            break;
        }
        
        // Mode once on the ground, ends flight logging
        case FSM_GROUNDED: {
            ARES.stopLogging();
            ARES.stopAllSensorThreads();

            // sleep trap after grounded to preserve power
            while (true) { ThisThread::sleep_for(1s); } 
        }

        // Mode once within target radius
        case FSM_SPIRAL: {
            float cmd = state.fc_cmd > 0 ? 1 : -1;
            ARES.setLastFCcmd(cmd);
            ARES.sendCtrl(cmd);
            break; 
        }

        // Unidentified Case
        default: break; 
    }
}



int main() {
    // Start CLI in the background
    // Thread cli_thread(osPriorityLow);
    Thread cli_thread;
    cli_thread.start(callback(&cli, &CLI::run));

    // ARES boot sequence
    ARES.startAllSensorThreads();
    ARES.startLogging();

    // wait for stable altitude measurement
    ThisThread::sleep_for(5s);
    ARES.updateFlightPacket();  // update altitude
    ARES.setThreshold();        // set altitude threshold for apogee detection

    Timer execution_timer;
    execution_timer.start();

    while (true) {
        execution_timer.reset();
        
        executeFlightLogic(); // the whole shabang
        
        // scheduling
        auto elapsed_time = execution_timer.elapsed_time();
        if (elapsed_time < MAIN_EXECUTION_PERIOD) {
            ThisThread::sleep_for(
                chrono::duration_cast<Kernel::Clock::duration>(
                    MAIN_EXECUTION_PERIOD - elapsed_time
                )
            );
        }
    }
}
