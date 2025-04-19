#ifndef GPS_H
#define GPS_H
#include "mbed.h"



#define KNOT_TO_M_S 0.5144444444

struct posECEFg {   // Earth-Centered Earth-Fixed Geodic Coordinates
    float lat; // lattitude    (radians)
    float lon; // longitude    (radians)
    float alt; // altitude     (meters)
};


struct posECEFr {  // Earth-Centered Earth-Fixed Rectangular Coordinates
    float x;       // get origin of linear tangent plan in ECEF-r coordinates
    float y;       // (will likely be the launch pad)
    float z;
};

struct posLTP { // Local Tangent Plane Coordinates
    float e;   // east     (m)
    float n;   // north    (m)
    float u;   // up       (m)
};
 

struct gpsState{
    float lat;
    float lon;
    float alt;
    float pdop;
    float vdop;
    float hdop;
    float heading; 
    float gspeed;
    float utc;
    char latNS;
    char lonEW;
    int fix;
    char rmcStatus;
    int date;
    char mode1;
    char mode2;
};


typedef enum {
    NMEA_NA,    // 0
    NMEA_GGA,   // 1
    NMEA_GSA,   // 2
    NMEA_GSV,   // 3
    NMEA_RMC,   // 4
    NMEA_VTG    // 5
} NMEA_Type;


typedef enum {
    chillin,    // 0
    uhoh,       // 1
    uhoh2,      // 2
    womp,       // 3
    notype      // 4
} gpsDebug;


class GPS {
    private:
        gpsState state;
        posLTP pos;
        posECEFr origin;
        float dms2rad(float coord_dms);
        int getLatSign();
        int getLonSign();

        // NMEA message readers
        int update_GGA(const char* msg);
        int update_GSA(const char* msg);
        int update_GSV(const char* msg);
        int update_RMC(const char* msg);
        int update_VTG(const char* msg);
        
        // getMsgType()
        // update()
    
    public:
        GPS(PinName rx_gps, PinName tx_gps);
        gpsState getState() const;
        NMEA_Type getMsgType(const char* msg); // TODO: make private and make wrapper
        int update(NMEA_Type msgType, const char* msg); // TODO: make private and make wrapper
        int bigUpdate();
        BufferedSerial serial;
        void setOriginECEFr(); // uses current position to set origin if nothing is passed
        void setOriginECEFr(posECEFr o); // can specify origin
        posLTP getPosLTP();

        // functino: start (GPS LOOOP)
        // initialize buffered serial object
        // make wrapper function for getMsgType() & update()
};

#endif // GPS_H