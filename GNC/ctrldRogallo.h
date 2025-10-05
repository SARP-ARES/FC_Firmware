#ifndef ROGALLO_H
#define ROGALLO_H
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "EUSBSerial.h"
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
        BMP280 bmp;
        BNO055 bno;
        FlightPacket state;
        ModeFSM mode;
        uint32_t apogeeDetected;
        uint32_t apogeeCounter;
        float alphaAlt;
        uint32_t groundedCounter; 
        float target_lat;
        float target_lon;
        float haversineCoordNorth;
        float haversineCoordEast;
        float distanceToTarget;
        posLTP ltp;
        uint32_t apogeeThreshold;
        uint32_t groundedThreshold; 

        float computeHaversine(double lat_deg, double lon_deg, double lat_target_deg, double lon_target_deg);
        void updateDistanceToTarget(void);
        void updateHaversineCoords(void);
        bool isWithinTarget(void);
        void setModeFSM(ModeFSM mode);
        float getFuzedAlt(float alt1, float alt2);
        void setAlphaAlt(float newAlphaAlt);
        void updateApogeeDetection();
        // string getCompassDirection(float rollMag, float pitchMag);

    public:
        ctrldRogallo();
        GPS gps;
        void setThreshold(); 
        void setTarget(double latitude, double longitude);
        uint32_t currentFlashAddress; // move to private after testing
        void updateFlightPacket();
        void resetFlightPacket();
        float computeCtrl(float thetaErr); // output in [-1, 1]
        void sendCtrl(float ctrl);
        void requestMotorPacket(void);
        uint32_t apogeeDetection(double prevAlt, double currAlt);
        uint32_t groundedDetection(double prevAlt, double currAlt);
        void logData();
        void logDataTEST();
        const FlightPacket getState();
        void printCompactState(EUSBSerial* pc);
};


#endif