#ifndef MUTEX_I2C_H
#define MUTEX_I2C_H

/* 
 * Wrapper class:
 *
 * Uses MBED api to safely mutex I2C calls over the bus 
 * 
 */

#include <cstdint>
#include "mbed.h"
#include "USBSerial.h"
using namespace std; 

class Mutex_I2C {

    public:

        /** @brief constructs I2C object */
        Mutex_I2C(PinName SDA, PinName SCL);

        // State
        bool owned; 

        // Functions
        int readData(char addr, char regAddr, char* data, uint8_t len);
        int writeData(char addr, char regAddr, char data);

    private:
        I2C i2c; // i2c api owns a mutex  
};

#endif 

