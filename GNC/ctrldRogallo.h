#ifndef ROGALLO_H
#define ROGALLO_H
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "flight_packet.h"
#include <cstdint>

// Finite State Machine Modes
typedef enum {
    FSM_IDLE,       // 0
    FSM_SEEKING,    // 1
    FSM_SPIRAL,     // 2
    FSM_GROUNDED,   // 3
} ModeFSM;



class ctrldRogallo {

    private:
        float getThetaErr();
        float getTargetHeading();
        GPS gps;
        BMP280 bmp;
        BNO055 bno;
        FlightPacket state;
        ModeFSM mode;
        bool apogeeDetected;
        uint32_t apogeeCounter;
        float alphaAlt;
        posLTP ltp;
        
        void setModeFSM(ModeFSM mode);
        float getFuzedAlt(float alt1, float alt2);
        void setAlphaAlt(float newAlphaAlt);
        void updateApogeeDetection();
        // string getCompassDirection(float rollMag, float pitchMag);

    public:
        uint32_t currentFlashAddress; // move to private after testing
        ctrldRogallo();
        void updateFlightPacket();
        void resetFlightPacket();
        float computeCtrl(float thetaErr); // output in [-1, 1]
        void sendCtrl(float ctrl);
        uint32_t apogeeDetection(double prevAlt, double currAlt);
        void logData();
        void logDataTEST();
        const FlightPacket getState();
};


#endif