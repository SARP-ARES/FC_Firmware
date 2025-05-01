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
    apogeeDetected = false;
    apogeeCounter = 0;
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


/**
 * @brief updates the state of the system to log as a packet of data
 */ 
void ctrldRogallo::updateFlightPacket(){
    
    gps_state = gps.getState();
    bmp_state = bmp.getState(); 
    posLTP ltp = gps.getPosLTP();

    double prevAlt = bmp_state.altitude_m;
    bmp.updateValues();
    int success = gps.bigUpdate(); 


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
    state.altitude_m = getFuzedAlt();
    state.pos_east_m = ltp.e;
    state.pos_north_m = ltp.n;
    state.pos_up_m = ltp.u;

    // BMP 
    state.temp_c = bmp_state.temp_c;
    state.pressure_pa = bmp_state.press_pa;
    state.apogee_detected = apogeeDetected;

    strncpy(state.flight_id, "BIKE01", sizeof(state.flight_id));

    // BNO
    state.yaw_rate = bno.getGyroscope().x; // Confirmed
    state.pitch_rate = bno.getGyroscope().y; // Confirmed
    state.roll_rate = bno.getGyroscope().z;  // Confirmed
    // state.compassDirecton = getCompassDirection(bno.getMagnetometer().z, bno.getMagnetometer().y);


    apogeeCounter += apogeeDetection(prevAlt, state.altitude_m);
    if(apogeeCounter >= 20){
        apogeeDetected = true;
        // trigger seeking mode
        mode = FSM_SEEKING; // 1
    }
    
}

/**
 * @Brief fuzes the altitude of the GPS and BMP reading using a complimentary filter
 * @return - the fuzed altitude of the two sensors
 */ 
float ctrldRogallo::getFuzedAlt(){
    float fuzedAlt = NAN; 
    if(!isnan(gps_state.alt) && !isnan(bmp_state.altitude_m)){
        fuzedAlt = bmp_state.altitude_m*(1-alphaAlt) + gps_state.alt*alphaAlt;
    } else if(isnan(gps_state.alt)){
        fuzedAlt = bmp_state.altitude_m;
    }else if(isnan(bmp_state.altitude_m)){
        fuzedAlt = gps_state.alt; 
    } else {
        fuzedAlt = NAN; 
    }
    return fuzedAlt;
}

void ctrldRogallo::setAlphaAlt(float newAlphaAlt){
    alphaAlt = newAlphaAlt;
}

/** 
 * @breif - detects if rocket has reached apogee based upon current velocity (-1.5 m/s constitutes as apogee)
 * @param prevAlt - previous altitude 
 * @param currAlt - current altitude
 * @return 0 if non apogee 1 if apogee
 */ 
int ctrldRogallo::apogeeDetection(double prevAlt, double currAlt){
    if(isnan(prevAlt) || isnan(currAlt)){
        return 0; 
    }
    double interval = .1;       // 0.1 seconds (10Hz)
    double apogeeVelo = -1.5;   // m/s
    double velo = (currAlt - prevAlt)/interval;
    if(velo <= apogeeVelo && currAlt >= 600){ // 600m threshold altitude
        return 1;
    }
    return 0; 
}

string ctrldRogallo::getCompassDirection(float rollMag, float pitchMag){
    float heading = atan2(rollMag, pitchMag) * 180/pi;
    if(heading < 0) heading += 360; 
    if(heading > 360) heading -= 360;
    if(heading > 360-22.5 || heading <= 22.5 ) {
        return "N";
    } 
    if(heading < 67.5){
        return "NE";
    }
    if(heading < 117.5){
        return "E";
    }
    if(heading < 167.5 ){
        return "SE";
    }
    if(heading < 217.5){
        return "S";
    }
    if(heading < 267.5){
        return "SW";
    }
    if(heading < 317.5){
        return "W";
    }
    return "NW";
}   

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

