#include "ctrldRogallo.h"
#include <cmath>
#include "EUSBSerial.h"
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include <string>

#define DEG_LLA_TO_M_CONVERSION         111111
#define APOGEE_THRESHOLD_BUFFER         600
#define GROUNDED_THRESHOLD_BUFFER       100
#define ALPHA_ALT_START_PERCENT         0.05
#define SPIRAL_RADIUS                   10
#define PI                              3.1415926535
#define DEG_TO_RAD                      PI/180.0

/**
 * @brief constructor that initializes the sensors and flash chip on the ARES flight computer.
 */ 
ctrldRogallo::ctrldRogallo() 
    : gps(PA_2, PA_3), bmp(PB_7, PB_8, 0xEE), bno(PB_7, PB_8, 0x51) {
    bmp.start();
    bno.setup();

    apogeeDetected = 0; // false
    apogeeCounter = 0;
    groundedCounter = 0; 

    //target lat/lon
    target_lat = NAN;
    target_lon = NAN;

    groundedThreshold = NAN;
    apogeeThreshold = NAN; 

    alphaAlt = ALPHA_ALT_START_PERCENT; // used to determine complimentary filter preference (majority goes to BMP)
    mode = FSM_IDLE; // initialize in idle mode
} 

/**
 * @brief getter for the current state of the system
 * @returns system state as a FlightPacket struct
 */ 
const FlightPacket ctrldRogallo::getState() {
    return this->state;
}

/**
 * @brief sets the seeking/landing target latitude & longitude
 * @param lat - target latitude
 * @param lon - target longitude
 */ 
void ctrldRogallo::setTarget(double lat, double lon) { target_lat = lat; target_lon = lon; }


/**
 * @brief cumputes the greater-circle distance between 
          two lat/lon coordinate pairs using the haversine formula
 * @param lat1_deg - latitude of first coordinate pair in degrees
 * @param lon1_deg - longitude of first coordinate pair in degrees
 * @param lat2_deg - latitude of second coordinate pair in degrees
 * @param lon2_deg - longitude of second coordinate pair in degrees
 */ 
float ctrldRogallo::computeHaversine(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    double dLat = (lat1_deg - lat2_deg) * DEG_TO_RAD;
    
    double dLon = (lon1_deg - lon2_deg) * DEG_TO_RAD;

    // convert to radians
    double lat_rad = lat1_deg * DEG_TO_RAD;
    double target_lat_rad = lat2_deg * DEG_TO_RAD;

    // apply formulae
    double a = pow(sin(dLat / 2), 2) + 
                pow(sin(dLon / 2), 2) * cos(lat_rad) * cos(target_lat_rad);

    double R = 6371000; // Average earth radius in meters

    return 2 * R * asin(sqrt(a)); // return in meters
}


/**
 * @brief Uses computeHaversine to convert the current lat/lon position into 
          meters east and north of the target and updates distance to target
 */ 
void ctrldRogallo::updateHaversineCoords(void){
    // only compute distance between latitudes to get NORTH coord
    haversineCoordNorth = computeHaversine(state.latitude_deg, target_lon, target_lat, target_lon);
    // make negative if south of target
    if (state.latitude_deg < target_lat)  { 
        haversineCoordNorth = -1 * haversineCoordNorth; 
    }

    // only compute distance between longitudes to get EAST coord
    haversineCoordEast = computeHaversine(target_lat, state.longitude_deg, target_lat, target_lon);
    // make negative if west of target
    if (state.longitude_deg < target_lon)  { 
        haversineCoordEast = -1 * haversineCoordEast; 
    }

    // update distance to target field
    distanceToTarget = computeHaversine(state.latitude_deg, state.longitude_deg, target_lat, target_lon);
}

bool ctrldRogallo::isWithinTarget(void) { return distanceToTarget < SPIRAL_RADIUS; }

/*  Python Code
def getTargetHeading(e, n):
    targetHeading_rad = np.arctan2(e,n) + np.pi
    targetHeading_deg = targetHeading_rad * 180 / np.pi
    return targetHeading_deg

def getThetaErr(actualHeading_deg, targetHeading_deg):
    error_deg = actualHeading_deg - targetHeading_deg 
    if error_deg > 180:
        error_deg -= 360
    elif error_deg < -180:
        error_deg += 360
    return error_deg
*/

/**
 * @brief calculates target heading to point towards the origin in local tangent plane coords
 * @return target heading 
 */ 
float ctrldRogallo::getTargetHeading(){
    float targetHeading_rad = atan2(state.pos_east_m, state.pos_north_m) + pi;
    float targetHeading_deg = targetHeading_rad * 180 / pi;
    return targetHeading_deg;
}

/**
 * @brief calculates heading error to feed into controller
 * @return heading error (current - target)
 */ 
float ctrldRogallo::getThetaErr(){
    float thetaErr_deg = this->state.heading_deg - this->state.target_heading_deg;
    if (thetaErr_deg > 180){
        // if its greater than 180 deg, add 360 deg
        thetaErr_deg = thetaErr_deg - 360;
    }else if (thetaErr_deg < -180) {
        // if its less than -180 deg, add 360 deg
        thetaErr_deg = thetaErr_deg + 360;
    }
    return thetaErr_deg;
}

void ctrldRogallo::resetFlightPacket() {
    // Set all float fields to NAN
    state.timestamp_utc      = NAN;
    state.heading_deg        = NAN;
    state.target_heading_deg = NAN;
    state.h_speed_m_s        = NAN;
    state.v_speed_m_s        = NAN;
    state.latitude_deg       = NAN;
    state.longitude_deg      = NAN;
    state.altitude_gps_m     = NAN;
    state.altitude_bmp_m     = NAN;
    state.altitude_m         = NAN;
    state.pos_east_m         = NAN;
    state.pos_north_m        = NAN;
    state.temp_c             = NAN;
    state.pressure_pa        = NAN;
    state.delta_1_deg        = NAN;
    state.delta_1_m          = NAN;
    state.delta_2_deg        = NAN;
    state.delta_2_m          = NAN;
    state.delta_a            = NAN;
    state.delta_s            = NAN;
    state.pwm_motor1         = NAN;
    state.pwm_motor2         = NAN;
    state.fc_cmd             = NAN;
    state.yaw_rate           = NAN;
    state.pitch_rate         = NAN;
    state.roll_rate          = NAN;
    
    // BNO055 sensor fields
    state.bno_acc_x          = NAN;
    state.bno_acc_y          = NAN;
    state.bno_acc_z          = NAN;
    state.bno_mag_x          = NAN;
    state.bno_mag_y          = NAN;
    state.bno_mag_z          = NAN;
    state.bno_eul_x          = NAN;
    state.bno_eul_y          = NAN;
    state.bno_eul_z          = NAN;
    state.bno_lin_x          = NAN;
    state.bno_lin_y          = NAN;
    state.bno_lin_z          = NAN;
    state.bno_grav_x         = NAN;
    state.bno_grav_y         = NAN;
    state.bno_grav_z         = NAN;
    state.bno_quat_w         = NAN;
    state.bno_quat_x         = NAN;
    state.bno_quat_y         = NAN;
    state.bno_quat_z         = NAN;

    // Set all integer fields to 0xFF (invalid/unknown)
    state.fsm_mode           = 0xFF;
    state.gps_fix            = 0xFFFF;
    state.apogee_counter     = 0xFFFFFFFF;
    state.apogee_detected    = 0xFF;

    // Set flight_id to empty string (or fill with 0xFF if you prefer)
    // std::memset(state.flight_id, 0, sizeof(state.flight_id));
}



/**
 * @brief updates the state of the system to log as a packet of data
 */ 
void ctrldRogallo::updateFlightPacket(){
    BMP280_Values bmp_state = bmp.getState();
    state.prevAlt = bmp_state.altitude_m;
    
    bmp.updateValues();
    int success = gps.bigUpdate(); 

    gpsState gps_state = gps.getState();
    bmp_state = bmp.getState(); 


    // GPS 
    state.timestamp_utc = gps_state.utc;
    state.fsm_mode = this->mode;
    state.gps_fix = gps_state.fix;
    state.heading_deg = gps_state.heading;
    state.target_heading_deg = getTargetHeading();
    state.h_speed_m_s = gps_state.gspeed;
    // state.h_speed_m_s = getHSpeed();
    state.latitude_deg = gps_state.lat;
    state.longitude_deg = gps_state.lon;
    state.altitude_gps_m = gps_state.alt;
    state.altitude_bmp_m = bmp_state.altitude_m;
    state.altitude_m = bmp_state.altitude_m;
    // state.altitude_m = getFuzedAlt(bmp_state.altitude_m, gps_state.alt); // shit don't work

    updateHaversineCoords();
    state.pos_east_m = haversineCoordEast;
    state.pos_north_m = haversineCoordNorth;
    state.distance_to_target_m = distanceToTarget;
    

    // BMP 
    state.temp_c = bmp_state.temp_c;
    state.pressure_pa = bmp_state.press_pa;

    // strncpy(state.flight_id, "BIKE01", sizeof(state.flight_id));

    // BNO 
    bno055_vector_t acc = bno.getAccelerometer();
    state.bno_acc_x = acc.x;
    state.bno_acc_y = acc.y;
    state.bno_acc_z = acc.z;

    bno055_vector_t gyr = bno.getGyroscope();
    state.yaw_rate = gyr.x;
    state.pitch_rate = gyr.y;
    state.roll_rate = gyr.z;

    bno055_vector_t mag = bno.getMagnetometer();
    state.bno_mag_x = mag.x;
    state.bno_mag_y = mag.y;
    state.bno_mag_z = mag.z;

    bno055_vector_t eul = bno.getEuler();
    state.bno_eul_x = eul.x;
    state.bno_eul_y = eul.y;
    state.bno_eul_z = eul.z;

    bno055_vector_t lin = bno.getLinearAccel();
    state.bno_lin_x = lin.x;
    state.bno_lin_y = lin.y;
    state.bno_lin_z = lin.z;

    bno055_vector_t grav = bno.getGravity();
    state.bno_grav_x = grav.x;
    state.bno_grav_y = grav.y;
    state.bno_grav_z = grav.z;

    bno055_vector_t quat = bno.getQuaternion();
    state.bno_quat_w = quat.w;
    state.bno_quat_x = quat.x;
    state.bno_quat_y = quat.y;
    state.bno_quat_z = quat.z;


    // state.compassDirecton = getCompassDirection(bno.getMagnetometer().z, bno.getMagnetometer().y);

    apogeeCounter += apogeeDetection(state.prevAlt, state.altitude_m); // checks if descending and above threshold

    if(apogeeCounter >= 5) apogeeDetected = 1;
    
    if(apogeeDetected == 1) {
        if(isWithinTarget())    mode = FSM_SPIRAL; // checks if ARES is within spiral target range mode = FSM_SPIRAL; 
        else                    mode = FSM_SEEKING;

        groundedCounter += groundedDetection(state.prevAlt, state.altitude_m); // checks if not moving and below threshold
        if(groundedCounter >= 15)    mode = FSM_GROUNDED;
    }

    state.apogee_counter = apogeeCounter;
    state.apogee_detected = apogeeDetected;
    state.groundedCounter = groundedCounter;
}

/**
 * @brief fuzes the altitude of the GPS and BMP reading using a complimentary filter
 * @return - the fuzed altitude of the two sensors
 */ 
float ctrldRogallo::getFuzedAlt(float alt1, float alt2){
    float fuzedAlt = NAN; 
    // check for NANs (they will not equal themselves)
    if (alt1 == alt1 && alt2 == alt2) {
        fuzedAlt = alt1*alphaAlt + alt2*(1-alphaAlt);
    } else if (alt1 == alt1) {
        fuzedAlt = alt1;
    }else if (alt2 == alt2){
        fuzedAlt = alt2; 
    } else {
        fuzedAlt = NAN; // both bmp and gps are giving nans
    }
    return fuzedAlt;
}

void ctrldRogallo::setAlphaAlt(float newAlphaAlt){
    alphaAlt = newAlphaAlt;
}

/** 
 * @brief - detects if rocket has reached apogee based upon current velocity (-1.5 m/s constitutes as apogee)
 * @param prevAlt - previous altitude 
 * @param currAlt - current altitude
 * @return 0 if non apogee 1 if apogee
 */ 
uint32_t ctrldRogallo::apogeeDetection(double prevAlt, double currAlt){
    double interval = 1; // seconds
    // double apogeeVelo = -1.5; // m/s
    double apogeeVelo = -1.2; // m/s
    double velo = (currAlt - prevAlt)/interval;
    if(velo <= apogeeVelo && currAlt > apogeeThreshold) { // REMINDER TO ADD --> 600m threshold altitude
        return 1;
    } 
    return 0; 
}

uint32_t ctrldRogallo::groundedDetection(double prevAlt, double currAlt) {
    double interval = 1; 
    double velo = (currAlt - prevAlt)/interval;
    if (velo < 0.3 && velo > -0.3 && currAlt < groundedThreshold) {
        return 1; 
    }
    return 0; 
}

void ctrldRogallo::setThreshold(){
    groundedThreshold = state.altitude_m + GROUNDED_THRESHOLD_BUFFER;
    apogeeThreshold = state.altitude_m + APOGEE_THRESHOLD_BUFFER; 
}

void ctrldRogallo::printCompactState(EUSBSerial* pc) {
    pc->printf("Lat (deg), Lon (deg), Alt (m):\t%f, %f, %.3f\n", 
                state.latitude_deg, state.longitude_deg, state.altitude_m);
    pc->printf("Pos North (m), Pos East (m):\t%.2f, %.2f\n", 
                state.pos_north_m, state.pos_east_m);
    pc->printf("Distance to Target (m):\t\t%.2f\n", state.distance_to_target_m);
    pc->printf("FSM mode:\t\t\t%d\n", state.fsm_mode);
    pc->printf("Apogee Counter:\t\t\t%d\n", state.apogee_counter);
    pc->printf("Apogee Detected:\t\t%d\n", state.apogee_detected);
    pc->printf("Grounded Counter:\t\t%d \n", state.groundedCounter);
    pc->printf("==========================================================\n");
}

// string ctrldRogallo::getCompassDirection(float rollMag, float pitchMag){
//     float heading = atan2(rollMag, pitchMag) * 180/pi;
//     if(heading < 0) heading += 360; 
//     if(heading > 360) heading -= 360;
//     if(heading > 360-22.5 || heading <= 22.5 ) {
//         return "N";
//     } 
//     if(heading < 67.5){
//         return "NE";
//     }
//     if(heading < 117.5){
//         return "E";
//     }
//     if(heading < 167.5 ){
//         return "SE";
//     }
//     if(heading < 217.5){
//         return "S";
//     }
//     if(heading < 267.5){
//         return "SW";
//     }
//     if(heading < 317.5){
//         return "W";
//     }
//     return "NW";
// }   

// /**
//  * @brief logs current state as a flight packet to the flash chip
//  */
// void ctrldRogallo::logData() {
//     currentFlashAddress = fc.writePacket(currentFlashAddress, state);
// }

// /**
//  * @brief logs current state as a flight packet to address 0 only
//  */
// void ctrldRogallo::logDataTEST() {
//     currentFlashAddress = 0;
//     currentFlashAddress = fc.writePacket(currentFlashAddress, state);
// }

