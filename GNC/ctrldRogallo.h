#ifndef ROGALLO_H
#define ROGALLO_H
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "flight_packet.h"

// Finite State Machine Modes
typedef enum {
    FSM_IDLE,       // 0
    FSM_SEEKING,    // 1
    FSM_SPIRAL,     // 2
    FSM_GROUNDED,   // 3
} ModeFSM;

// struct gpsState{ // Do I have to define this here again??
//     double lat;
//     double lon;
//     double alt;
//     double hdop;
//     double heading; 
//     double gspeed;
//     double utc;
//     char latNS;
//     char lonEW;
//     int fix;
// };

// struct bmpState{ // Do I have to define this here again??
//     double alt;
//     double press;
//     double temp;
// };

// struct imuState{ // Do I have to define this here again??
//     double theta_deg;
//     double x;
    
// };


class ctrldRogallo {
    
const float pi = 3.14159265359;

    private:
        float getThetaErr();
        float getTargetHeading();
        GPS gps;
        BMP280 bmp;
        BNO055 bno;
        flash fc;
        FlightPacket state;
        ModeFSM mode;
        bool apogeeDetected;
        int apogeeCounter;
        float alphaAlt;
        BMP280_Values bmp_state;
        gpsState state_gps;
        posLTP ltp;
        void setModeFSM(ModeFSM mode);
        float getFuzedAlt();
        void setAlphaAlt(float newAlphaAlt);
        void updateApogeeDetection();

    public:
        ctrldRogallo();
        void updateFlightPacket();
        float computeCtrl(float thetaErr); // output in [-1, 1]
        void sendCtrl(float ctrl);
        int apogeeDetection(double prevAlt, double currAlt);
};


#endif