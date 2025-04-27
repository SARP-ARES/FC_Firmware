#include "ctrldRogallo.h"
#include <cmath>
#include "GPS.h"

ctrldRogallo::ctrldRogallo() : gps(PA_2, PA_3), bmp(PB_7, PB_8, 0xEE), bno(PB_7, PB_8, 0x51), fc(PA_7, PA_6, PA_5, PA_4) {
    apogeeDetected = false;
    apogeeCounter = 0;
    alphaAlt = .05; // used to determine complimentary filter preference  
    mode = FSM_IDLE; // idle
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

float ctrldRogallo::getTargetHeading(){
    float targetHeading_rad = atan2(state.pos_east_m, state.pos_north_m) + pi;
    float targetHeading_deg = targetHeading_rad * 180 / pi;
    return targetHeading_deg;
}

float ctrldRogallo::getThetaErr(){
    float thetaErr_deg = this->state.actHeading - this->state.actTarget;
    if (thetaErr_deg > 180){
        // if its greater than 180 deg, add 360 deg
        thetaErr_deg = thetaErr_deg - 360;
    }else if (thetaErr_deg < -180) {
        // if its less than -180 deg, add 360 deg
        thetaErr_deg = thetaErr_deg + 360;
    }
    return thetaErr_deg;
}



void ctrldRogallo::updateFlightPacket(){

    

    state_gps = gps.getState();
    bmp_state = bmp.getState(); 
    posLTP ltp = gps.getPosLTP();

    double prevAlt = bmp_state.altitude_m;
    bmp.updateValues();
    gps.bigUpdate(); 

    
    // complementary kalman filter here
    // update actual and target heading

    state.timestamp_utc = state_gps.utc;
    state.fsm_mode = this->mode;
    state.gps_fix = state_gps.fix;
    state.heading_deg = state_gps.heading;
    state.target_heading_deg = getTargetHeading();
    // state.groundspeed_m_s = getVSpeed();
    // state.h_speed_m_s = getHSpeed();
    state.latitude_deg = state_gps.lat;
    state.longitude_deg = state_gps.lon;
    state.altitude_m = getFuzedAlt();
    state.pos_east_m = ltp.e;
    state.pos_north_m = ltp.n;
    state.pos_up_m = ltp.u;
    state.temp_c = bmp_state.temp_c;
    state.pressure_pa = bmp_state.press_pa;
    apogeeCounter += apogeeDetection(prevAlt, bmp_state.altitude_m);
    if(apogeeCounter >= 20){
        apogeeDetected = true; 
    }
    
    this->state.actHeading = 0;
    this->state.actTarget = 0;
}

/**
 * @Brief fuzes the altitude of the GPS and BMP reading using a complimentary filter
 * @return - the fuzed altitude of the two sensors
**/ 
float ctrldRogallo::getFuzedAlt(){
    float fuzedAlt; 
    if(isnan(state_gps.alt) && isnan(bmp_state.altitude_m)){
        fuzedAlt = -1; 
    } else if(isnan(state_gps.alt)){
        fuzedAlt = bmp_state.altitude_m;
    }else if(isnan(bmp_state.altitude_m)){
        fuzedAlt = state_gps.alt; 
    } else {
        fuzedAlt = bmp_state.altitude_m*(1-alphaAlt) + state_gps.alt*alphaAlt;
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
**/ 
int ctrldRogallo::apogeeDetection(double prevAlt, double currAlt){
    if(isnan(prevAlt) || isnan(currAlt)){
        return 0; 
    }

    double interval = .1; 
    double apogeeVelo = -1.5;
    double velo = (currAlt - prevAlt)/interval;
    if(velo <= apogeeVelo){
        return 1;
    }
    return 0; 
}

