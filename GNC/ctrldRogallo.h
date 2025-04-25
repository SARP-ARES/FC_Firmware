#ifndef ROGALLO_H
#define ROGALLO_H
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"


struct filteredState{
    double latOrigin_deg;
    double lonOrigin_deg;
    double posNorth;
    double posEast;
    double posUp;
    double actHeading;
    double targetHeading;
    double gSpeed;
    double descRate;
};

struct gpsState{ // Do I have to define this here again??
    double lat;
    double lon;
    double alt;
    double hdop;
    double heading; 
    double gspeed;
    double utc;
    char latNS;
    char lonEW;
    int fix;
};

struct bmpState{ // Do I have to define this here again??
    double alt;
    double press;
    double temp;
};

struct imuState{ // Do I have to define this here again??
    double theta_deg;
    double x;
    
};

const float pi = 3.14159265359;

class ctrldRogallo {
    private:
        filteredState state;
        float getThetaErr();
        float getTargetHeading();
        GPS gps;
        BMP280 bmp;
        BNO055 bno;

    public:
        ctrldRogallo();
        filteredState getState() const;
        void updateState(gpsState* stateGPS, bmpState* stateBMP, imuState* stateIMU);
        float computeCtrl(float thetaErr); // output in [-1, 1]
        void sendCtrl(float ctrl);
        

};


#endif