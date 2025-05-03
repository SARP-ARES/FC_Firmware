// Class which translates a float input (from the flight controller) into two floats for each 
// motors linear displacement
#ifndef _DISTRIBUTOR_H_
#define _DISTRIBUTOR_H_


#include "mbed.h"
#include "I2CSerial.h"

#define DISTRIBUTOR_BUFFER_SIZE 8

class Distributor {
    private:
        I2CSerial* ser;

    public:
        Distributor();

        std::pair<float, float> getMotorOutputs();
};


#endif // _DISTRIBUTOR_H_