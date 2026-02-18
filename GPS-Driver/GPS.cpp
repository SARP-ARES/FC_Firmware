#include "BufferedSerial.h"
#include "EUSBSerial.h"
#include "GPS.h"
#include "GPS_CMD.h"
#include <cmath>


/*

ADD DESCRIPTION

*/
GPS::GPS(PinName rx_gps, PinName tx_gps, EUSBSerial& externalUSBSerial) : serial(rx_gps, tx_gps), eSerialDebug(externalUSBSerial) {}



/*

Peter's set Loggin rate method

*/
void GPS::set_logging_rate(uint32_t hz) {
    // Choose valid GPS settings based on requested Hz
    const GPSCmd* baudCmd = nullptr; // GPSCmd* baudCmd means the var "baudCmd" just stores the memory address for the acutal GPSCmd object
    const GPSCmd* updateCmd = nullptr;
    const GPSCmd* fixCmd = nullptr; //check if * placement does anything
    const GPSCmd* nemaCmd = &CMD_NMEA_NECESSARY;
    uint32_t uart_baud = 0;


    if (hz <= 1) {
        baudCmd = &CMD_BAUD_9600; // & menas to grab the memory adress of the given variable/object. 
        updateCmd = &CMD_UPDATE_1HZ;
        fixCmd = &CMD_FIXCTL_1HZ;
        uart_baud = 9600;
    } else if (hz <= 5) {
        baudCmd = &CMD_BAUD_38400;
        updateCmd = &CMD_UPDATE_5HZ;
        fixCmd = &CMD_FIXCTL_5HZ;
        uart_baud = 38400;
    } else { // default: 10 Hz
        baudCmd = &CMD_BAUD_38400;  // GPS stable at 10Hz with 38400 (CHECK WHETER TO USE THIS OR 115200)
        updateCmd = &CMD_UPDATE_10HZ;
        fixCmd = &CMD_FIXCTL_10HZ;
        uart_baud = 38400;
    }

    

    // Step 1: Set GPS baud rate
    this->serial.write(baudCmd->cmd, baudCmd->len);
    eSerialDebug.printf("\nBuad CMD should have sent\n");
    ThisThread::sleep_for(100ms); //are these okay or can we not set the thread to sleep (other threads need it)

    // Step 2: Update STM32 UART baud to match GPS
    this->serial.set_baud(uart_baud);
    eSerialDebug.printf("\nUART baud CMD should have sent\n");
    ThisThread::sleep_for(100ms);

    // Step 3: Set GPS update rate
    this->serial.write(updateCmd->cmd, updateCmd->len);
    eSerialDebug.printf("\nUpdate CMD should have sent\n");
    ThisThread::sleep_for(100ms);

    // Step 4: Set GPS fix interval
    this->serial.write(fixCmd->cmd, fixCmd->len);
    eSerialDebug.printf("\nFix CMD should have sent\n");
    ThisThread::sleep_for(100ms);

    // Step 5: Set NEMA sentence types
    this->serial.write(nemaCmd->cmd, nemaCmd->len);
    eSerialDebug.printf("\nNEMA CMD should have sent\n");
    ThisThread::sleep_for(100ms);

}



/*
ADD DESCRIPTION
*/
gpsState GPS::getState() const{
    return state; // return a copy of the state (can't be modified bc its private)
}


/*
ADD DESCRIPTION
*/
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
        case ',': { // PCD (antenna status)
            return NMEA_ANT;
        }  
    }
    return NMEA_NA; // No type found
}


/*
ADD DESCRIPTION
*/
int GPS::getLatSign() {
    int sign = 1; // expect northern hemisphere
    if (state.latNS == 'S') {
        sign = -1;
    }
    return sign;
}

/*
ADD DESCRIPTION
*/
int GPS::getLonSign() {
    int sign = -1; // expect western longitude 
    if (state.latNS == 'E') {
        sign = 1;
    }
    return sign;
}


float GPS::lat2deg(float lat_ddmm){
    int lat_sign = getLatSign(); // -1 or 1
    float lat_deg = floor(lat_ddmm/100);       // extract degrees
    float lat_min = fmod(lat_ddmm, 100);     // extract minutes
    // convert degrees & minutes to radians
    lat_deg = lat_sign*(lat_deg + lat_min/60); 
    return lat_deg;
}

float GPS::lon2deg(float lon_dddmm){
    int lon_sign = getLonSign(); // -1 or 1
    float lon_deg = floor(lon_dddmm/100);       // extract degrees
    float lon_min = fmod(lon_dddmm, 1000);     // extract minutes
    // convert degrees & minutes to radians
    lon_deg = lon_sign*(lon_deg + lon_min/60); 
    return lon_deg;
}


/*
converts degrees to radians
*/
float GPS::deg2rad(float deg) {
    float rad = deg * pi/180; 
    return rad;
}


/**
 * @brief converts UTC time in hhmmss.sss to seconds
 * @param utc - time in UTC with forma hhmmss.sss
 * @return time in seconds (UTC)
 */
float GPS::utc2sec(float utc) {
    // hhmmss.sss
    int hours = (int)(utc / 10000);
    int minutes = (int)((utc - hours * 10000) / 100);
    float seconds = utc - hours * 10000 - minutes * 100;
    float total_seconds = hours * 3600 + minutes * 60 + seconds;
    return total_seconds;
}


/*

ADD DESCRIPTION

*/
int GPS::update_GGA(const char* msg){
    int _;
    char subtype = 'O';
    float utc = NAN;
    float lat = NAN;
    char latNS = 'O';
    float lon = NAN;
    char lonEW = 'O';
    int fix = 404;
    int nsats = -1;
    float alt = NAN;
    float hdop = NAN;
    int result = -999;
    
    // messages can be GNGGA (GNSS fix) or GPGGA (GPS fix)
    result = sscanf(msg, "$G%cGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f", &subtype, &utc, &lat, &latNS, &lon, \
                        &lonEW, &fix, &nsats, &hdop, &alt);
              
    // assign values to the state
    this->state.utc = utc2sec(utc);
    this->state.lat = lat2deg(lat);
    this->state.latNS = latNS;
    this->state.lon = lon2deg(lon);
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

int GPS::update_GSA(const char* msg){
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

int GPS::update_RMC(const char* msg){
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
    this->state.utc = utc2sec(utc);
    this->state.rmcStatus = status;
    this->state.lat = lat2deg(lat);
    this->state.latNS = latNS;
    this->state.lon = lon2deg(lon);
    this->state.lonEW = lonEW;
    this->state.gspeed = gspeed*KNOT_TO_M_S; // convert knots to m/s
    this->state.heading = heading;
    this->state.date = date;

    return result;
}

int GPS::update_VTG(const char* msg){
    return 0;
}

int GPS::update_antenna_status(const char* msg){
    uint8_t antenna_status = 0;
    // Parse the RMC message
    int result = sscanf(msg, "$PCD,11,%d",
                        &antenna_status);
    this->state.antenna_status = antenna_status;
    
    return result;
}


int GPS::update(NMEA_Type msgType, const char* msg){

    switch(msgType) {
        
        case NMEA_NA: { // type not recognized
            // printf("Invalid NMEA message type\n");
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

        case NMEA_ANT: {
            int result = update_antenna_status(msg);
        }
    }
    return 0;
}


/*

ADD DESCRIPTION

*/
void GPS::setOriginECEFr() { // uses current position to set origin if nothing is passed
    // get longitude and latitude into radians
    float lat_rad = state.lat * pi/180;
    float lon_rad = state.lon * pi/180;

    const float a = 6378137; // earth semi-major axis        (m)
    const float b = 6356752.3142; // earth semi-minor axis   (m)
    const float f = (a-b)/a; // ellipsoid flatness 
    const float e = sqrt(f*(2-f)); // eccentricity
    // distance from the earth's surface to the z-axis along the ellipsoid normal
    const float N = a/sqrt( 1 - pow(e, 2)* pow(sin(lat_rad), 2) ); 

    // get current position in ECEF-r coordinates
    float h = state.alt;
    float x = (h + N)*cos(lat_rad)*cos(lon_rad);
    float y = (h + N)*cos(lat_rad)*sin(lon_rad);
    float z = (h + N*(1 - pow(e,2)))*sin(lat_rad);
    origin.x = x;
    origin.y = y;
    origin.z = z;
}


const float a = 6378137; // earth semi-major axis        (m)
const float b = 6356752.3142; // earth semi-minor axis   (m)
const float f = (a-b)/a; // ellipsoid flatness 
const float e = sqrt(f*(2-f)); // eccentricity

/*
Overloaded... 
can specify the lattitude and longitude of the origin in degrees if needed
instead of pulling from the current GPS position
*/
void GPS::setOriginECEFr(float lat_deg, float lon_deg, float h) { // uses current position to set origin if nothing is passed
    // convert longitude and latitude into radians
    float lat_rad = lat_deg * pi/180;
    float lon_rad = lon_deg * pi/180;
 
    // distance from the earth's surface to the z-axis along the ellipsoid normal
    const float N = a/sqrt( 1 - pow(e, 2)* pow(sin(lat_rad), 2) ); 

    // get current position in ECEF-r coordinates
    // float h = state.alt;
    float x = (h + N)*cos(lat_rad)*cos(lon_rad);
    float y = (h + N)*cos(lat_rad)*sin(lon_rad);
    float z = (h + N*(1 - pow(e,2)))*sin(lat_rad);
    origin.x = x;
    origin.y = y;
    origin.z = z;
} 




void GPS::updatePosLTP() {
    // state.lat = ddmm.mmmm ... state.lat/100 = dd.mmmmmm
    // North = positive, South = negative
    // East = positive, West = negative 
    
    // get longitude and latitude from degrees into radians
    float lat_rad = deg2rad(state.lat);
    float lon_rad = deg2rad(state.lon);

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

    // // final rotation to allign z_axis with up
    // pos.e = x_pp;
    // pos.n = -y_pp*sin(lat_rad) + z_pp*cos(lat_rad);
    // pos.u = y_pp*cos(lat_rad) + z_pp*sin(lat_rad);

    // TESTING RELATIVE POSITION IN ECEFr
    pos.e = x_p;
    pos.n = y_p;
    pos.u = z_p;
}


int GPS::bigUpdate(){
    // multiple messages come in... gotta parse all of them and then update the state
    Timer t;
    t.start();

    updatePosLTP(); // update local tangent plane position

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

        if (t.read_ms() > 2000) { // something fishy is going on
            break;
        }
    }
    return success; // number of messages processed with result > 0 (matched some items)
}