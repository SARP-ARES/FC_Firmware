#include "EUSBSerial.h"
#include "GPS.h"
#include <cmath>

GPS::GPS() {}

gpsState GPS::getState() const{
    return state; // return a copy of the state (can't be modified bc its private)
}

// say const so that it can't be modified (only reading it)
// msg will be the entire line 
NMEA_Type GPS::getMsgType(const char* msg) {

    switch (msg[4]) {
        case 'G': { // GGA
            return NMEA_GGA; // Might need NMEA_Type::NMEA_GGA;
            break;
        }
        case 'S': { // GSA
            return NMEA_GSA;
            break;
        }
        case 'M': { // RMC
            return NMEA_RMC;
            break;
        }
        case 'T': { // VTG
            return NMEA_VTG;
            break;
        }
    }
    return NMEA_NA; // No type found
}

/*
typedef enum {
    NMEA_NA,    // 0
    NMEA_GGA,   // 1
    NMEA_GSA,   // 2
    NMEA_RMC,   // 3
    NMEA_VTG    // 4
} NMEA_Type;
*/


int GPS::update_GGA(const char* msg){ // TODO: NEEDS TESTING
    int _;
    double utc;
    double lat;
    char latNS;
    double lon;
    char lonEW;
    int fix;
    int nsats;
    double alt;
    double hdop;

    int result = sscanf(msg, "$GPGGA,%lf,%lf,%c,%lf,%c,%d,%d,%lf,%lf,%c,%lf,%c,%lf,*%x", &utc, &lat, &latNS, &lon, \
                        &lonEW, &fix, &nsats, &hdop, &alt, &_, &_, &_, &_, &_);
     
    // int result = sscanf(msg, "$GPGGA,%lf,%lf,%c,%lf,%c,%d,%d,%lf,%lf,%c,%lf,%c,%lf,*%x", &utc, &lat, &latNS, &lon, \
    //                     &lonEW, &fix, &nsats, &hdop, &alt, &altUnits, &gsep, &gsepUnits, &ageCorrection, &checksum);
            
    // assign values to the state
    if (result > 0) {
        this->state.utc = utc;
        this->state.lat = lat;
        this->state.latNS = latNS;
        this->state.lon = lon;
        this->state.lonEW = lonEW;
        this->state.fix = fix;
        this->state.hdop = hdop;
        this->state.alt = alt;
        if (result == 14) {
            // If parsing was successful
            printf("State Succesfully Updated");
            // logState()
        }
        return result;

    } else if (result == 0) {
        // log failure
        printf("Failed to parse GGA message\n");
        return result;
    } else {
        // matched some stuff
        return result;
    }
}

int GPS::update_GSV(const char* msg){ // TODO: NEEDS TESTING
    // Process GSV Message and Update State
    int numMessages, messageNum, satellitesInView;
    int satelliteIDs[4] = {0}; // Up to 4 satellites per message
    int elevations[4] = {0};   // Elevations for up to 4 satellites
    int azimuths[4] = {0};     // Azimuths for up to 4 satellites
    int snrs[4] = {0};         // Signal-to-noise ratios for up to 4 satellites
    int checksum;

    // Parse the GSV message
    int result = sscanf(msg, "$GPGSV,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,*%x",
                        &numMessages, &messageNum, &satellitesInView,
                        &satelliteIDs[0], &elevations[0], &azimuths[0], &snrs[0],
                        &satelliteIDs[1], &elevations[1], &azimuths[1], &snrs[1],
                        &satelliteIDs[2], &elevations[2], &azimuths[2], &snrs[2],
                        &checksum);

    // Assign values to the state
    if (result > 0) {
        this->state.numMessages = numMessages;
        this->state.messageNum = messageNum;
        this->state.satellitesInView = satellitesInView;
        for (int i = 0; i < 4; i++) {
            this->state.satelliteIDs[i] = satelliteIDs[i];
            this->state.elevations[i] = elevations[i];
            this->state.azimuths[i] = azimuths[i];
            this->state.snrs[i] = snrs[i];
        }
        return result;

        if (result >= 15) {
            printf("State Successfully Updated\n");
        } else if (result == 0) {
            printf("Failed to parse GSV message\n");
            return result;
        } else {
            return result;
        }
    }
}

int GPS::update_GSA(const char* msg){ // TODO: NEEDS TESTING
    return 0;
}

int GPS::update_RMC(const char* msg){ // TODO: NEEDS TESTING
    double utc;
    char status;
    double lat;
    char latNS;
    double lon;
    char lonEW;
    double gspeed;
    double heading;
    int date;
    double magneticVariation = 0.0; // Optional field
    char mode = '\0'; // Optional field
    int checksum;

    // Parse the RMC message
    int result = sscanf(msg, "$GPRMC,%lf,%c,%lf,%c,%lf,%c,%lf,%lf,%d,%lf,%c*%x",
                        &utc, &status, &lat, &latNS, &lon, &lonEW,
                        &gspeed, &heading, &date,
                        &magneticVariation, &mode, &checksum);

    // Assign values to the state
    if (result > 0) {
        this->state.utc = utc;
        this->state.status = status;
        this->state.lat = lat;
        this->state.latNS = latNS;
        this->state.lon = lon;
        this->state.lonEW = lonEW;
        this->state.gspeed = gspeed;
        this->state.heading = heading;
        this->state.date = date;

        if (result >= 10) { // Magnetic variation and mode are optional
            this->state.magneticVariation = magneticVariation;
            this->state.mode = mode;
        }

        return result;

        if (result >= 10) {
            printf("State Successfully Updated\n");
        }
        } else if (result == 0) {
            printf("Failed to parse RMC message\n");
            return result;
        } else {
            return result;
        }
}

int GPS::update_VTG(const char* msg){ // TODO: NEEDS TESTING
    return 0;
}


// Q: should I call getMsgType inside update() or leave them separate?
int GPS::update(int msgType, const char* msg){

    // initialize state variables
    int _;
    double utc;
    double lat;
    char latNS;
    double lon;
    char lonEW;
    int fix;
    double alt;
    double hdop;
    double heading; 
    double gspeed;

    // currently unused vars
    int nsats;
    char altUnits;
    double gsep;
    char gsepUnits;
    double ageCorrection;
    int checksum;

    switch (msgType) {
        
        case NMEA_NA: { // type not recognized
            printf("Invalid NMEA message type\n");
            break;
            return notype;
        }

        case NMEA_GGA: { // GGA
            
            int result = update_GGA(msg);
            return result;
            // int result = sscanf(msg, "$GPGGA,%lf,%lf,%c,%lf,%c,%d,%d,%lf,%lf,%c,%lf,%c,%lf,*%x", &utc, &lat, &latNS, &lon, \
            //             &lonEW, &fix, &nsats, &hdop, &alt, &altUnits, &gsep, &gsepUnits, &ageCorrection, &checksum);
            
            // // assign values to the state
            // if (result > 0) {
            //     this->state.utc = utc;
            //     this->state.lat = lat;
            //     this->state.latNS = latNS;
            //     this->state.lon = lon;
            //     this->state.lonEW = lonEW;
            //     this->state.fix = fix;
            //     this->state.hdop = hdop;
            //     this->state.alt = alt;
            //     if (result == 14) {
            //         // If parsing was successful
            //         printf("State Succesfully Updated");
            //         // logState()
            //     }
            //     return result;

            // } else if (result == 0) {
            //     // log failure
            //     printf("Failed to parse GGA message\n");
            //     return result;
            // } else {
            //     // matched some stuff
            //     return result;
            // }
        }

        case NMEA_GSA: { // GSA]
            return womp;
            // char mode1;
            // int mode2;
            // int satellites[12] = {0}; // Up to 12 satellites used
            // double pdop, hdop, vdop;
            // int checksum;

            // // Parse the GSA message
            // int result = sscanf(msg, "$GPGSA,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lf,%lf,%lf*%x",
            //                     &mode1, &mode2,
            //                     &satellites[0], &satellites[1], &satellites[2], &satellites[3],
            //                     &satellites[4], &satellites[5], &satellites[6], &satellites[7],
            //                     &satellites[8], &satellites[9], &satellites[10], &satellites[11],
            //                     &pdop, &hdop, &vdop, &checksum);

            // // Assign values to the state
            // if (result > 0) {
            //     this->state.mode1 = mode1;
            //     this->state.mode2 = mode2;
            //     for (int i = 0; i < 12; i++) {
            //         this->state.satellites[i] = satellites[i];
            //     }
            //     this->state.pdop = pdop;
            //     this->state.hdop = hdop;
            //     this->state.vdop = vdop;
            //     return result;

            //     if (result == 17) {
            //         printf("State Successfully Updated\n");
            //     }
            // } else if (result == 0) {
            //     printf("Failed to parse GSA message\n");
            //     return result;
            //     break;
            // } else {
            //     return result;
            // }
        }


        case NMEA_RMC: { // RMC
            // use sscanf to update state
            return womp;
        }

        case NMEA_VTG: { // VTG
            // use sscanf to update state
            return womp;
        }
    }
    return womp;
}

// posECEFg GPS::getPosECEFg() { 

//     posECEFg z = state.lat

//     return posECEFg;
// }

posLTP GPS::getPosLTP() {
    // state.lat = ddmm.mmmm ... state.lat/100 = dd.mmmmmm
    // Determine lat/lon coords as North or South & East or West
    int lat_sign;
    int lon_sign;
    switch(state.latNS) {
        case('N'): {
            lat_sign = 1;
            break;
        }
        case('S'): {
            lat_sign = -1;
            break;
        }
        printf("something is super wrong... latNS = '%c'", state.latNS);
    }
    switch(state.lonEW) {
        case('E'): {
            lon_sign = 1;
            break;
        }
        case('W'): {
            lon_sign = -1;
            break;
        }
        printf("something is super wrong... latEW = '%c'", state.lonEW);
    }

    double pi = 3.1415926535898;

    // get longitude and latitude into radians
    int lat_deg = floor(state.lat/100);
    double lat_min = fmod(state.lat, 100);
    double lat_rad = lat_sign*( (lat_deg + lat_min/60) * pi/180 );

    int lon_deg = floor(state.lat/1000);
    double lon_min = fmod(state.lat, 1000);
    double lon_rad = lon_sign*( (lon_deg + lon_min/60) * pi/180 );

    
    double a = 6378137; // earth semi-major axis        (m)
    double b = 6356752.3142; // earth semi-minor axis   (m)
    double f = (a-b)/a; // ellipsoid flatness 
    double e = sqrt(f*(2-f)); // eccentricity
    // distance from the earth's surface to the z-axis along the ellipsoid normal
    double N = a/sqrt( 1 - pow(e, 2)* pow(sin(lat_rad), 2) ); 

    // get current position in ECEF-r coordinates
    double h = state.alt;
    double x = (h + N)*cos(lat_rad)*cos(lon_rad);
    double y = (h + N)*cos(lat_rad)*sin(lon_rad);
    double z = (h + N*(1 - pow(e,2)))*sin(lat_rad);
    
    // subtract the origin ECEF-r coordinate to get relative position vector
    double x_p = x - origin.x;
    double y_p = y - origin.y;
    double z_p = z - origin.z;

    // rotate to allign y-axis w/ LTP
    double x_pp = -x_p*sin(lon_rad) + y_p*cos(lon_rad);
    double y_pp = x_p*cos(lon_rad) + y_p*sin(lon_rad);
    double z_pp = z_p;

    // final rotation to allign z_axis with up
    pos.e = x_pp;
    pos.n = -y_pp*sin(lat_rad) + z_pp*cos(lat_rad);
    pos.u = y_pp*cos(lat_deg) + z_pp*sin(lat_rad);

    return pos;
}