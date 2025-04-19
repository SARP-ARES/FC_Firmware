#include "BufferedSerial.h"
#include "EUSBSerial.h"
#include "GPS.h"
#include <cmath>

GPS::GPS(PinName rx_gps, PinName tx_gps) : serial(rx_gps, tx_gps) {}

gpsState GPS::getState() const{
    return state; // return a copy of the state (can't be modified bc its private)
}


// say "const" so that it can't be modified (only reading it)
// msg will be the entire line 
NMEA_Type GPS::getMsgType(const char* msg) {

    switch (msg[4]) {
        case 'G': { // GGA
            return NMEA_GGA; // Might need NMEA_Type::NMEA_GGA;
            break;
        }
        case 'S': { // GSA or GSV
            switch(msg[5]) {
                case 'A': { // GSA
                    return NMEA_GSA;
                    break;
                }
                case 'V': { // GSV
                    return NMEA_GSV;
                    break;
                }
            }
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


int GPS::getLatSign() {
    int sign = 1; // expect northern hemisphere
    if (state.latNS == 'S') {
        sign = -1;
    }
    return sign;
}

int GPS::getLonSign() {
    int sign = -1; // expect western longitude 
    if (state.latNS == 'E') {
        sign = 1;
    }
    return sign;
}

float GPS::dms2rad(float coord_dms) {
    // converts ddmm.mmmm to d (rad)
    float pi = 3.1415926535898;
    int coord_deg = floor(coord_dms/100);       // extract degrees
    float coord_min = fmod(coord_dms, 100);     // extract minutes
    // convert degrees & minutes to radians
    float coord_rad = coord_deg*( (coord_deg + coord_min/60) * pi/180 ); 
    return coord_rad;
}


int GPS::update_GGA(const char* msg){ // TODO: NEEDS TESTING
    int _;
    char subtype = 'O';
    float utc = NAN;
    float lat = NAN;
    char latNS = 'O';
    float lon = NAN;
    char lonEW = 'O';
    int fix = -1;
    int nsats = -1;
    float alt = NAN;
    float hdop = NAN;
    int result = -999;
    
    // messages can be GNGGA (GNSS fix) or GPGGA (GPS fix)
    result = sscanf(msg, "$G%cGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f", &subtype, &utc, &lat, &latNS, &lon, \
                        &lonEW, &fix, &nsats, &hdop, &alt);
              
    // assign values to the state
    this->state.utc = utc;
    this->state.lat = lat;
    this->state.latNS = latNS;
    this->state.lon = lon;
    this->state.lonEW = lonEW;
    this->state.fix = fix;
    this->state.hdop = hdop;
    this->state.alt = alt;

    return result;
}

int GPS::update_GSV(const char* msg){ // don't care
    //     // Process GSV Message and Update State
    //     int numMessages, messageNum, satellitesInView;
    //     int satelliteIDs[4] = {0}; // Up to 4 satellites per message
    //     int elevations[4] = {0};   // Elevations for up to 4 satellites
    //     int azimuths[4] = {0};     // Azimuths for up to 4 satellites
    //     int snrs[4] = {0};         // Signal-to-noise ratios for up to 4 satellites
    //     int checksum;

    //     // Parse the GSV message
    //     int result = sscanf(msg, "$GPGSV,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,*%x",
    //                         &numMessages, &messageNum, &satellitesInView,
    //                         &satelliteIDs[0], &elevations[0], &azimuths[0], &snrs[0],
    //                         &satelliteIDs[1], &elevations[1], &azimuths[1], &snrs[1],
    //                         &satelliteIDs[2], &elevations[2], &azimuths[2], &snrs[2],
    //                         &checksum);

    //     // Assign values to the state
    //     if (result > 0) {
    //         this->state.numMessages = numMessages;
    //         this->state.messageNum = messageNum;
    //         this->state.satellitesInView = satellitesInView;
    //         for (int i = 0; i < 4; i++) {
    //             this->state.satelliteIDs[i] = satelliteIDs[i];
    //             this->state.elevations[i] = elevations[i];
    //             this->state.azimuths[i] = azimuths[i];
    //             this->state.snrs[i] = snrs[i];
    //         }
    //         return result;

    //         if (result >= 15) {
    //             printf("State Successfully Updated\n");
    //         } else if (result == 0) {
    //             printf("Failed to parse GSV message\n");
    //             return result;
    //         } else {
    //             return result;
    //         }
    //     }
    return 0;
}

int GPS::update_GSA(const char* msg){ // TODO: NEEDS TESTING
    // GSA - GPS DOP and Active Satellites
    char subtype = 'O';
    char mode1 = 'O';           // Operating mode (M=Manual, A=Automatic)
    int mode2 = -1;             // Fix type (1=No fix, 2=2D, 3=3D)
    float pdop = NAN;
    float hdop = NAN;
    float vdop = NAN;
    int checksum = -1;
    int result = -999;
    
    // messages can be GLGSA (GLONASS) or GPGSA (GPS)
    // 1) use sscanf to get mode2 (fix type)
    // 2) loop through character array and count commas (17 total for GSA, dops start after 15)

    result = sscanf(msg, "$G%cGSA,%c,%d", &subtype, &mode1, &mode2);
    
    int commas = 0;
    int index = 0;
    char c;
    while(true) { // count to 15 commas and break 
        c = msg[index];
        if (c == ',') {
            commas++;
        } // count commas
        if (commas == 15){ // dops are after the 15th comma
            index++; // move to the next index after 15th comma
            break; // get out of while loop
        }
        index++;
    }
    const char* msgDops = &msg[index]; // gets the rest of msg AFTER the 15th comma

    int result2 = sscanf(msgDops, "%f,%f,%f*%x", &pdop, &hdop, &vdop, &checksum); 
    result = result + result2;

    // Assign values to the state
    this->state.mode1 = mode1;
    this->state.mode2 = mode2;
    this->state.pdop = pdop;
    this->state.hdop = hdop;
    this->state.vdop = vdop;

    return result;
}

int GPS::update_RMC(const char* msg){ // TODO: NEEDS TESTING
    float utc = NAN;
    char status = 'O';
    float lat = NAN;
    char latNS = 'O';
    float lon = NAN;
    char lonEW = 'O';
    float gspeed = NAN;
    float heading = NAN;
    int date = -1;
    float magneticVariation = NAN; // Optional field
    char mode = 'O'; // Optional field
    int checksum = -1;

    // Parse the RMC message
    int result = sscanf(msg, "$GNRMC,%f,%c,%f,%c,%f,%c,%f,%f,%d,%f,%c*%x",
                        &utc, &status, &lat, &latNS, &lon, &lonEW,
                        &gspeed, &heading, &date,
                        &magneticVariation, &mode, &checksum);

    // Assign values to the state
    this->state.utc = utc;
    this->state.rmcStatus = status;
    this->state.lat = lat;
    this->state.latNS = latNS;
    this->state.lon = lon;
    this->state.lonEW = lonEW;
    this->state.gspeed = gspeed*KNOT_TO_M_S; // convert knots to m/s
    this->state.heading = heading;
    this->state.date = date;

    return result;
}

int GPS::update_VTG(const char* msg){ // TODO: NEEDS TESTING
    return 0;
}


// Q: should I call getMsgType inside update() or leave them separate?
int GPS::update(NMEA_Type msgType, const char* msg){

    // initialize state variables
    int _;
    float utc;
    float lat;
    char latNS;
    float lon;
    char lonEW;
    int fix;
    float alt;
    float hdop;
    float heading; 
    float gspeed;

    // currently unused vars
    int nsats;
    char altUnits;
    float gsep;
    char gsepUnits;
    float ageCorrection;
    int checksum;

    switch(msgType) {
        
        case NMEA_NA: { // type not recognized
            printf("Invalid NMEA message type\n");
            return 0;
        }

        case NMEA_GGA: { // GGA
            int result = update_GGA(msg);
            return result;
        }

        case NMEA_GSA: { // GSA]
            int result = update_GSA(msg);
            return result;
        }

        case NMEA_GSV: { // GSA]
            // don't care
            return 0;
        }

        case NMEA_RMC: { // RMC
            int result = update_RMC(msg);
            return result;
        }

        case NMEA_VTG: { // VTG
            // don't care
            return 0;
        }
    }
    return 0;
}

int GPS::bigUpdate(){
    // multiple messages come in... gotta parse all of them and then update the state
    Timer t;
    t.start();

    char buf[256] = {0};
    int index = 0;
    
    int success = 0;
    bool GGA_processed = false;
    bool GSA_processed = false;
    bool GSV_processed = false;
    bool RMC_processed = false;
    bool VTG_processed = false;
    

    while (true) { // read messages and send data to vars
        if (serial.readable()) {
            serial.read(&buf[index], 1);
            index ++;
        }

        if (index != 0 && buf[index-1] == '\n') {
            buf[index] = 0; // terminate the string to negate leftovers
            NMEA_Type msgType = getMsgType(buf);
            int result = update(msgType, buf);
            switch(msgType) { // update bools
                case NMEA_GGA: {
                    GGA_processed = true;
                    break;
                }
                case NMEA_GSA: {
                    GSA_processed = true;
                    break;
                }
                case NMEA_GSV: {
                    GSV_processed = true;
                    break;
                }
                case NMEA_RMC: {
                    RMC_processed = true;
                    break;
                }
                case NMEA_VTG: {
                    VTG_processed = true;
                    break;
                }
            }

            if (result > 0){
                success++;
            }

            index = 0; // reset buffer index
        }

        if (GGA_processed && GSA_processed && RMC_processed && VTG_processed) {
            break; // break out once all message types have been processed
        }

        if (t.read_ms() > 5000) { // something fishy is going on
            break;
        }
    }
    return success; // number of messages processed with result > 0 (matched some items)
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

    float pi = 3.1415926535898;

    // get longitude and latitude into radians
    int lat_rad = dms2rad(state.lat);
    int lon_rad = dms2rad(state.lon);


    float a = 6378137; // earth semi-major axis        (m)
    float b = 6356752.3142; // earth semi-minor axis   (m)
    float f = (a-b)/a; // ellipsoid flatness 
    float e = sqrt(f*(2-f)); // eccentricity
    // distance from the earth's surface to the z-axis along the ellipsoid normal
    float N = a/sqrt( 1 - pow(e, 2)* pow(sin(lat_rad), 2) ); 

    // get current position in ECEF-r coordinates
    float h = state.alt;
    float x = (h + N)*cos(lat_rad)*cos(lon_rad);
    float y = (h + N)*cos(lat_rad)*sin(lon_rad);
    float z = (h + N*(1 - pow(e,2)))*sin(lat_rad);
    
    // subtract the origin ECEF-r coordinate to get relative position vector
    float x_p = x - origin.x;
    float y_p = y - origin.y;
    float z_p = z - origin.z;

    // rotate to allign y-axis w/ LTP
    float x_pp = -x_p*sin(lon_rad) + y_p*cos(lon_rad);
    float y_pp = x_p*cos(lon_rad) + y_p*sin(lon_rad);
    float z_pp = z_p;

    // final rotation to allign z_axis with up
    pos.e = x_pp;
    pos.n = -y_pp*sin(lat_rad) + z_pp*cos(lat_rad);
    pos.u = y_pp*cos(lat_rad) + z_pp*sin(lat_rad);

    return pos;
}