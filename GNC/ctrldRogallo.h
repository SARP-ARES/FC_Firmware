#ifndef GPS_H
#define GPS_H
// #include "mbed.h"
#include "GPS.h"

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

class ctrldRogallo {
    private:
        filteredState state;
        float getThetaErr();
        float getTargetHeading();

    public:
        filteredState getState() const;
        void updateState(gpsState stateGPS, bmpState stateBMP, imuState stateIMU);
        float computeCtrl(float thetaErr); // output in [-1, 1]
        void sendCtrl(float ctrl);
};


#endif