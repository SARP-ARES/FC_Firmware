#include "cli.h"
#include "flight_packet.h"

const uint32_t PRINT_FLAG = (1UL << 0);

/** @brief constructor for command-line interface class*/
CLI::CLI(EUSBSerial* pc_init,
        ctrldRogallo* ARES_init,
        flash* flash_chip_init,
        uint32_t* flash_addr_init)
        : 
        pc(pc_init),
        ARES(ARES_init),
        flash_chip(flash_chip_init),
        flash_addr(flash_addr_init)
        {
            // Start the thread state printing thread
            // It will immediately block and wait for PRINT_FLAG to be set
            print_state_thread.start(callback(this, &CLI::printStateLoop));
        }


/** @brief runs the command line interface*/
void CLI::run() {

    char cmd_buffer[32];
    ThisThread::sleep_for(2s);

    while (true) {

        if (this->print_menu) {
            printMenu();
            this->print_menu = false;
        }

        // reset buffer
        memset(cmd_buffer, 0, sizeof(cmd_buffer));

        // catch incoming commands
        if (pc->readline(cmd_buffer, sizeof(cmd_buffer))) {
            stopPrintingState();
            handleCommand(cmd_buffer);
        }
        
        ThisThread::sleep_for(100ms);
    }
}


/** @brief clears all data off of the flash chip */
void CLI::eraseData(){
    Timer t;
    t.start();
    pc->printf("\nErasing flash chip memory...\n\n");
    int erase_err = flash_chip->eraseAll(); 
    int ms = chrono::duration_cast<chrono::milliseconds>(t.elapsed_time()).count();
    ARES->resetPacketsLogged();
    if (erase_err == 0) {
        pc->printf("...Erasing Complete... (%d ms)\n\n", ms);
    } else if(erase_err == 1) {
        pc->printf("...Erasing timed out... something went wrong");
    }
}


/** @brief dumps all logged data in the serial port */
void CLI::dumpData(){
    uint16_t numPacketDump;
    flash_chip->read(0x3FFFFE, reinterpret_cast<uint8_t*> (&numPacketDump), 2);

    if (numPacketDump == 0xFFFF || numPacketDump == 0) {
        // If it's the default erased value, there are no packets to dump
        pc->printf("There are no packets to dump!");
        ThisThread::sleep_for(1500ms);

    } else {
        // big dumpy
        pc->printf("\nDumping %d packets...", numPacketDump);
        pc->printf("\n==================================\n");
        flash_chip->dumpAllPackets(numPacketDump);
        pc->printf("==================================\n");
    }

}


/** @brief make sure the user wants to clear all flash data before proceeding */
void CLI::doubleCheckErase() {
    char cmdBuf[32];
    pc->printf("Are you sure you want to erase all data from ARES?\n");
    pc->printf("1. \"yes\"\n2. \"no\"\n");
    
    Timer t;
    int ms;
    t.start();
    while (true) {
        if (pc->readline(cmdBuf, sizeof(cmdBuf))) {
            if (strcmp(cmdBuf, "yes") == 0 || strcmp(cmdBuf, "1") == 0) {
                pc->printf("\"yes\" received\n");
                ThisThread::sleep_for(1500ms);
                pc->printf("Ok... What's the password?\n");
                pc->printf("\n> ");
                ThisThread::sleep_for(2s);
                pc->printf("Just kidding :)\n");
                ThisThread::sleep_for(500ms);
                eraseData();
                break;
            } else if (strcmp(cmdBuf, "no") == 0 || strcmp(cmdBuf, "2") == 0) {
                pc->printf("\"no\" received\n");
                ThisThread::sleep_for(1500ms);
                break;
            } else if (strcmp(cmdBuf, "yo mama") == 0 || strcmp(cmdBuf, "3") == 0) {
                pc->printf("\"yo mama\" received\n");
                ThisThread::sleep_for(1500ms);
                pc->printf("That wasn't very nice :(");
                ThisThread::sleep_for(1s);
                break;
            }
        }
        // break (don't erase) if there is no response in 15 seconds
        ms = chrono::duration_cast<chrono::milliseconds>(t.elapsed_time()).count();
        if (ms > 15000) {
            pc->printf("You took too long! Try again...");
            break;
        }
    }
}


/** @brief Sets the target landing coordinates */
void CLI::setOrigin() {
    char cmd_buffer[32]; // user input buffer

    pc->printf("Where would you like to set the origin?\n");
    pc->printf("Enter \"here\" or \"<lat>, <lon>\"\n> ");

    // Wait for user input
    while (true) {
        if (pc->readline(cmd_buffer, sizeof(cmd_buffer))) {
            break;
        }
    }

    double lat, lon;
    if (strcmp(cmd_buffer, "here") == 0) {
        pc->printf("\"here\" received...\n");
        ThisThread::sleep_for(1500ms);
        
        // get current lat/lon and set target there
        ARES->updateFlightPacket();
        FlightPacket state = ARES->getState();

        if (state.latitude_deg != NAN && state.longitude_deg != NAN){
            ARES->setTarget(state.latitude_deg, state.longitude_deg);
            pc->printf("Origin has been set...\nLat, Lon: %f deg, %f deg\n", state.latitude_deg, state.longitude_deg);
            ThisThread::sleep_for(1500ms);
        } else { // lat/lon are nans... GPS doesn't have a fix yet.
            pc->printf("Origin has NOT been set because the GPS does not yet have fix.\n" \
                    "Make sure the antenna has a clear view of the sky and try again later.");
        }

    } else if (sscanf(cmd_buffer, "%lf, %lf", &lat, &lon) == 2) {
        // get lat/lon from user input and set as origin
        pc->printf("coordinates received...\n");
        ARES->setTarget(lat, lon);
        pc->printf("Origin has been set...\nLat, Lon: %lf deg, %lf deg\n", \
                    lat, lon);
        ThisThread::sleep_for(1500ms);
        

    } else {
        pc->printf("\nInvalid format. Use: \"<lat>, <lon>\" or type \"here\".\n");
    }
        
    // no lat/lon argument
    // sets current lat/lon as target/origin
    
}

void CLI::printCompactState() {
    // grab current state from ARES
    FlightPacket state = ARES->getState();

    pc->printf("Timer:\t\t\t\t\t%f s\n", state.timestamp_timer);
    pc->printf("Lat (deg), Lon (deg), Alt (m):\t\t%f, %f, %.3f\n", 
                state.latitude_deg, state.longitude_deg, state.altitude_m);
    pc->printf("Pos North (m), Pos East (m):\t\t%.2f, %.2f\n", 
                state.pos_north_m, state.pos_east_m);
    pc->printf("(Heading, deg) Current, Desired, Error:\t%.1f, %.1f, %.1f\n", 
                state.heading_deg, state.target_heading_deg, state.heading_error_deg);
    pc->printf("FC CMD:\t\t\t\t\t%.1f\n", state.fc_cmd); // TODO: implement PID 
    pc->printf("Motor 1 Position (in), EXT:\t\t%.4f, %0.3f\n", state.leftPosition, state.leftPull);
    pc->printf("Motor 2 Position (in), EXT:\t\t%.4f, %0.3f\n", state.rightPosition, state.rightPull);
    pc->printf("Distance to Target (m):\t\t\t%.2f\n", state.distance_to_target_m);

    const char* MODE_NAMES[] = {"IDLE", "SEEKING", "SPIRAL", "GROUNDED"};
    if (state.fsm_mode >= 0 && state.fsm_mode < 4) pc->printf("FSM mode:\t\t\t\t%s\n", MODE_NAMES[state.fsm_mode]);

    pc->printf("Temperature C: \t\t\t\t%lf\n", state.temp_c);
    pc->printf("Altitude M: \t\t\t\t%lf\n", state.altitude_bmp_m);
    pc->printf("Apogee Counter:\t\t\t\t%d\n", state.apogee_counter);
    pc->printf("Apogee Detected:\t\t\t%d\n", state.apogee_detected);
    pc->printf("Grounded Counter:\t\t\t%d \n", state.groundedCounter);
    pc->printf("==========================================================\n");
}

/**
* @brief prints the ARES state at 0.5-ish Hz
*/
 void CLI::printStateLoop() {
     while (true) {
         event_flags.wait_any(PRINT_FLAG, osWaitForever, false);
         printCompactState();
         ThisThread::sleep_for(2s); // ~0.5Hz
     }
 }

/**
* @brief sets the print_state flag to TRUE so the printStateLoop thread can run */
void CLI::startPrintingState() {
    event_flags.set(PRINT_FLAG);
}

/**
* @brief sets the print_state flag to FALSE so the printStateLoop thread does not run 
*/
void CLI::stopPrintingState() {
    event_flags.clear(PRINT_FLAG);
    ThisThread::sleep_for(100ms);
}


void CLI::printMenu(){
    pc->printf("\n\nARES is waiting for user input... What would you like to run?\n");
    pc->printf("1. \"dump\"\n");
    pc->printf("2. \"set_origin\"\n");
    pc->printf("3. \"clear\"\n");
    pc->printf("4. \"print_on\"\n");
    pc->printf("5. \"print_off\"\n");
    pc->printf("6. \"logging_off\"\n");
    pc->printf("7. \"logging_on\"\n");
    pc->printf("8. \"sensors_off\"\n");
    pc->printf("9. \"sensors_on\"\n");
    pc->printf("Force FSM transition with \"FSM_<mode>\"\n");
    pc->printf("\n> ");  // command prompt
}


void CLI::handleCommand(const char* cmd) {
    // =========================
    // dump data
    // =========================
    if (strcmp(cmd, "dump") == 0 || strcmp(cmd, "1") == 0) {
        pc->printf("\"dump\" cmd received\n");
        ThisThread::sleep_for(1500ms);
        dumpData();
        this->print_menu = true;
    }

    // =========================
    // set origin
    // =========================
    else if (strcmp(cmd, "set_origin") == 0 || strcmp(cmd, "2") == 0) {
        pc->printf("\"set_origin\" cmd received...\n");        
        ThisThread::sleep_for(1500ms);
        setOrigin();
        this->print_menu = true;
    }        

    // =========================
    // clear data
    // =========================
    else if (strcmp(cmd, "clear_data") == 0 || strcmp(cmd, "3") == 0) {
        pc->printf("\"clear_data\" cmd received\n");
        ThisThread::sleep_for(1500ms);
        doubleCheckErase(); // make sure the user wants to clear the data
        this->print_menu = true;
    }

    // =========================
    // print state over usb
    // =========================
    else if (strcmp(cmd, "print_on") == 0 || strcmp(cmd, "4") == 0)  {
        pc->printf("\"print_on\" received\n");
        ThisThread::sleep_for(100ms);
        startPrintingState();
    }

    // =========================
    // stop printing state
    // =========================
    else if (strcmp(cmd, "print_off") == 0 || strcmp(cmd, "5") == 0) {
        pc->printf("\"print_off\" cmd received\n");
        ThisThread::sleep_for(100ms);
        stopPrintingState();
        pc->printf("> ");
    }

    // =========================
    // stop logging
    // =========================
    else if (strcmp(cmd, "logging_off") == 0 || strcmp(cmd, "6") == 0) {
        pc->printf("\"logging_off\" cmd received\n> ");
        ThisThread::sleep_for(100ms);
        ARES->stopLogging();
    }

    // =========================
    // start logging
    // =========================
    else if (strcmp(cmd, "logging_on") == 0 || strcmp(cmd, "7") == 0) {
        pc->printf("\"logging_on\" cmd received\n> ");
        ThisThread::sleep_for(100ms);
        ARES->startLogging();
    }

    // =========================
    // stop sensor threads
    // =========================
    else if (strcmp(cmd, "sensors_off") == 0 || strcmp(cmd, "8") == 0) {
        pc->printf("\"sensors_off\" cmd received\n> ");
        ThisThread::sleep_for(100ms);
        ARES->stopAllSensorThreads();
    }

    // =========================
    // start sensor threads
    // =========================
    else if (strcmp(cmd, "sensors_on") == 0 || strcmp(cmd, "9") == 0) {
        pc->printf("\"sensors_on\" cmd received\n> ");
        ThisThread::sleep_for(100ms);
        ARES->startAllSensorThreads();
    }

    // =========================
    // FSM idle
    // =========================
    else if (strcmp(cmd, "FSM_idle") == 0 || strcmp(cmd, "M0") == 0) {
        pc->printf("\"FSM_idle\" cmd received\n> ");
        ThisThread::sleep_for(100ms);
        ARES->setFSMMode(FSM_IDLE);
    }

    // =========================
    // FSM seeking
    // =========================
    else if (strcmp(cmd, "FSM_seeking") == 0 || strcmp(cmd, "M1") == 0) {
        pc->printf("\"FSM_seeking\" cmd received\n> ");
        ThisThread::sleep_for(100ms);
        ARES->setFSMMode(FSM_SEEKING);
    }

    // =========================
    // FSM spiral
    // =========================
    else if (strcmp(cmd, "FSM_spiral") == 0 || strcmp(cmd, "M2") == 0) {
        pc->printf("\"FSM_spiral\" cmd received\n> ");
        ThisThread::sleep_for(100ms);
        ARES->setFSMMode(FSM_SPIRAL);
    }

    // =========================
    // FSM grounded
    // =========================
    else if (strcmp(cmd, "FSM_grounded") == 0 || strcmp(cmd, "M3") == 0) {
        pc->printf("\"FSM_grounded\" cmd received\n> ");
        ThisThread::sleep_for(100ms);
        ARES->setFSMMode(FSM_GROUNDED);
    }

    // =========================
    // hello
    // =========================
    else if (strcmp(cmd, "hello") == 0) {
        pc->printf("\"hello\" received\n");
        ThisThread::sleep_for(1500ms);
        pc->printf("\nhello! :)\n");
        this->print_menu = true;
    }

    // =========================
    // help
    // =========================
    else if (strcmp(cmd, "help") == 0) {
        pc->printf("\"help\" received\n");
        ThisThread::sleep_for(1500ms);
        this->print_menu = true;
    }

    // =========================
    // unknown command
    // =========================
    else {
        pc->printf("Unknown command: %s\n", cmd);
        this->print_menu = true;
    }
}

