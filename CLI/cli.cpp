#include "cli.h"
#include "flight_packet.h"
#include "run_mode.h"

// Forward declarations for flight functions that still live elsewhere
extern void testMode(ctrldRogallo* ARES, uint32_t* flash_addr);
extern void autoFlight(ctrldRogallo* ARES, uint32_t* flash_addr);
extern void runMode(RunMode mode, ctrldRogallo* ARES);


/** @brief constructor */
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
        }


/** @brief runs the command line interface*/
void CLI::run() {

    char cmd_buffer[32];
    ThisThread::sleep_for(2s);

    bool print_menu = true;

    while (true) {

        if (print_menu) {
            printMenu();
            print_menu = false;
        }

        memset(cmd_buffer, 0, sizeof(cmd_buffer));

        if (pc->readline(cmd_buffer, sizeof(cmd_buffer))) {
            handleCommand(cmd_buffer);
            print_menu = true;
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
    int ms = t.read_ms(); 
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
        if (t.read_ms() > 15000) {
            pc->printf("You took too long! Try again...");
            break;
        }
    }
}


/** @brief Sets the target landing coordinates */
void CLI::setOrigin() {
    char cmd_buffer[32]; // user input buffer

    pc->printf("Where would you like to set the origin?\n");
    pc->printf("Enter \"here\" or \"lat, lon\"\n> ");

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
            pc->printf("Origin has been set...\nLat, Lon: %f deg, %f deg\n", \
                    state.latitude_deg, state.longitude_deg);
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


void CLI::printMenu(){
    pc->printf("\n\nARES is waiting for user input... What would you like to run?\n");
    pc->printf("0. \"flight_mode\"\n");
    pc->printf("1. \"test_mode\"\n");
    pc->printf("2. \"dump\"\n");
    pc->printf("3. \"set_origin\"\n");
    pc->printf("4. \"clear\"\n");
    pc->printf("5. \"dump_to_ser\"");
    pc->printf("\n> ");  // command prompt
}


void CLI::handleCommand(const char* cmd) {
    // =========================
    // test mode
    // =========================
    if (strcmp(cmd, "test_mode") == 0 || strcmp(cmd, "1") == 0) {
        pc->printf("\"test_mode\" cmd received.\n");
        pc->printf("Use \"quit\" to stop logging.\n");
        pc->printf("Use \"seeking\", \"idle\", or \"spiral\" to force FSM.\n");
        ThisThread::sleep_for(1500ms);
        runMode(RunMode::Test, ARES);
    }

    // =========================
    // flight mode
    // =========================
    if (strcmp(cmd, "flight_mode") == 0 || strcmp(cmd, "6") == 0) {
        pc->printf("\"flight_mode\" cmd received.");
        ThisThread::sleep_for(1500ms);
        runMode(RunMode::Flight, ARES);
    }

    // =========================
    // dump data
    // =========================
    else if (strcmp(cmd, "dump") == 0 || strcmp(cmd, "2") == 0) {
        pc->printf("\"dump\" cmd received\n");
        ThisThread::sleep_for(1500ms);
        dumpData();
    }

    // =========================
    // dump data (30s delay)
    // =========================
    else if (strcmp(cmd, "dump_to_ser") == 0 || strcmp(cmd, "5") == 0) {
        pc->printf("\"dump_to_ser\" cmd received\n");
        pc->printf("Waiting 30s, disconnect from serial port and start serial parser\n");
        ThisThread::sleep_for(30s);
        dumpData();
    }

    // =========================
    // set origin
    // =========================

    else if (strcmp(cmd, "set_origin") == 0 || strcmp(cmd, "3") == 0) {

        pc->printf("\"set_origin\" cmd received...\n");
        ThisThread::sleep_for(1500ms);
        setOrigin();
    }        

    // =========================
    // clear data
    // =========================
    else if (strcmp(cmd, "clear") == 0 || strcmp(cmd, "4") == 0) {
        pc->printf("\"clear\" cmd received\n");
        ThisThread::sleep_for(1500ms);
        doubleCheckErase(); // make sure the user wants to clear the data
    }

    // =========================
    // hello
    // =========================
    else if (strcmp(cmd, "hello") == 0) {
        pc->printf("\"hello\" received\n");
        ThisThread::sleep_for(1500ms);
        pc->printf("\nhello! :)\n");
    }

    // =========================
    // unknown command
    // =========================
    else {
        pc->printf("Unknown command: %s\n", cmd);
    }

}

