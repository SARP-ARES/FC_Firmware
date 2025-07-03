#include "ctrldRogallo.h"
#include <cmath>
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include <string>


/**
 * @brief constructor that initializes the sensors and flash chip on the ARES flight computer.
 */ 
ctrldRogallo::ctrldRogallo() 
    : gps(PA_2, PA_3), bmp(PB_7, PB_8, 0xEE), bno(PB_7, PB_8, 0x51) {
    bmp.start();
    bno.setup();

    

    apogeeDetected = 0; // false
    apogeeCounter = 0;
    isGrounded = 0; 

    groundedThreshold = NAN;
    apogeeThreshold = NAN; 

    alphaAlt = .05; // used to determine complimentary filter preference (majority goes to BMP)
    mode = FSM_IDLE; // initialize in idle mode
    currentFlashAddress = 0; // start writing to address zero TODO: maybe pass as param?
} 


const FlightPacket ctrldRogallo::getState() {
    return this->state;
}

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

#include <cmath>    // for NAN
#include <cstring>  // for memset, strcpy

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
    state.pos_up_m           = NAN;
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
    posLTP ltp = gps.getPosLTP();

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
    state.pos_east_m = ltp.e;
    state.pos_north_m = ltp.n;
    state.pos_up_m = ltp.u;

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

    apogeeCounter += apogeeDetection(state.prevAlt, state.altitude_m);
    if(apogeeCounter >= 5) { 
        apogeeDetected = 1; // true
        // trigger seeking mode
        mode = FSM_SEEKING; // 1
    } 
    
    if(apogeeDetected == 1) {
        isGrounded += groundedDetection(state.prevAlt, state.altitude_m);
    }

    state.apogee_counter = apogeeCounter;
    state.apogee_detected = apogeeDetected;
    state.groundedCounter = isGrounded;
    if(isGrounded >= 15){
        mode = FSM_GROUNDED; 
    }
}

/**
 * @Brief fuzes the altitude of the GPS and BMP reading using a complimentary filter
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
    groundedThreshold = state.altitude_m + 100;
    apogeeThreshold = state.altitude_m + 600; 
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

