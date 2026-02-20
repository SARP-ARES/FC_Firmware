#ifndef CLI_H
#define CLI_H

#include "mbed.h"
#include "EUSBSerial.h"
#include "ctrldRogallo.h"
#include "flash.h"

class CLI {
public:
    CLI(EUSBSerial* serial,
        ctrldRogallo* flight,
        flash* flash_chip,
        uint32_t* flash_addr);

    void run();   // start CLI loop

private:
    EUSBSerial* pc;
    ctrldRogallo* ARES;
    flash* flash_chip;
    uint32_t* flash_addr;

    void printMenu();
    void handleCommand(const char* cmd);

    // commands
    void dumpData();
    void doubleCheckErase();
    void eraseData();
    void setOrigin();
};

#endif
