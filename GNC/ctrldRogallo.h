#ifndef ROGALLO_H
#define ROGALLO_H
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "flight_packet.h"
#include "I2CSerial.h"
#include <cstdint>

// Finite State Machine Modes
typedef enum {
    FSM_IDLE,       // 0
    FSM_SEEKING,    // 1
    FSM_SPIRAL,     // 2
    FSM_GROUNDED,   // 3
} ModeFSM;

#pragma pack(push, 1) // remove padding to ensure correct memory layout
struct MotorData {
    float delta1_deg;       // 4 
    float delta2_deg;       // 4
    float pwm_motor1;       // 4
    float pwm_motor2;       // 4
};
#pragma pack(pop)

const float SPOOL_DIAM = 15.77 / 1000; // 15.77mm
const float SPOOL_CIRC = SPOOL_DIAM * pi;

class ctrldRogallo {

    private:
        float getThetaErr();
        float getTargetHeading();
        GPS gps;
        BMP280 bmp;
        BNO055 bno;
        I2CSerial fc_mcps;
        FlightPacket state;
        ModeFSM mode;
        bool apogeeDetected;
        uint32_t apogeeCounter;
        float alphaAlt;
        posLTP ltp;
        MotorData getMotorData();
        void setModeFSM(ModeFSM mode);
        float getFuzedAlt(float alt1, float alt2);
        void setAlphaAlt(float newAlphaAlt);
        char* getCompassDirection();

    public:
        uint32_t currentFlashAddress; // move to private after testing
        ctrldRogallo();
        void updateFlightPacket();
        void resetFlightPacket();
        float computeCtrl(float thetaErr); // output in [-1, 1]
        void sendCtrl(float ctrl);
        uint32_t apogeeDetection(double prevAlt, double currAlt);
        const FlightPacket getState();
};


#endif