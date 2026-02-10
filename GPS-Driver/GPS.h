#ifndef GPS_H
#define GPS_H
#include "mbed.h"


const float KNOT_TO_M_S = 0.5144444444;


struct GPSData{
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
    uint8_t antenna_status;
};


typedef enum {
    NMEA_NA,    // 0
    NMEA_GGA,   // 1
    NMEA_GSA,   // 2
    NMEA_GSV,   // 3
    NMEA_RMC,   // 4
    NMEA_VTG,   // 5
    NMEA_ANT    // 6 
} NMEA_Type;


typedef enum {
    chillin,    // 0
    uhoh,       // 1
    uhoh2,      // 2
    womp,       // 3
    notype      // 4
} gpsDebug;

const float pi = 3.1415926535898;

class GPS {
    private:
        GPSData state;
        int getLatSign();
        int getLonSign();
        float utc2sec(float utc);

        // NMEA message readers
        int update_GGA(const char* msg);
        int update_GSA(const char* msg);
        int update_GSV(const char* msg);
        int update_RMC(const char* msg);
        int update_VTG(const char* msg);
        int update_antenna_status(const char* msg);

        void updatePosLTP();
        
        // getMsgType()
        // update()
    
    public:
        GPS(PinName rx_gps, PinName tx_gps);
        GPSData getData() const;
        float deg2rad(float deg);
        float lat2deg(float lat_ddmm);
        float lon2deg(float lon_dddmm);
        NMEA_Type getMsgType(const char* msg); // TODO: make private and make wrapper
        int update(NMEA_Type msgType, const char* msg); // TODO: make private and make wrapper
        int bigUpdate();
        BufferedSerial serial;
        mutable Mutex mutex;
        void setOriginECEFr(); // uses current position to set origin if nothing is passed
        void setOriginECEFr(float lat_deg, float lon_deg, float h); // can specify origin
        

        // functino: start (GPS LOOOP)
        // initialize buffered serial object
        // make wrapper function for getMsgType() & update()
};

#endif // GPS_H