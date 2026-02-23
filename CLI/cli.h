#ifndef CLI_H
#define CLI_H

#include "mbed.h"
#include "EUSBSerial.h"
#include "ctrldRogallo.h"
#include "flash.h"
#include "flight_packet.h"

class CLI {
public:
    CLI(EUSBSerial* serial,
        ctrldRogallo* flight,
        flash* flash_chip,
        uint32_t* flash_addr);

    void run();   // start CLI loop

    // thread business
    void startPrintingState();
    void stopPrintingState();

private:
    EUSBSerial* pc;
    ctrldRogallo* ARES;
    flash* flash_chip;

    EventFlags event_flags;
    Thread print_state_thread;
    void printCompactState();
    void printStateLoop();

    bool print_menu = false;

    uint32_t* flash_addr;

    void printMenu();
    void handleCommand(const char* cmd);

    // commands
    void dumpData();
    void doubleCheckErase();
    void eraseData();
    void setOrigin();
};

#endif // CLI_H
