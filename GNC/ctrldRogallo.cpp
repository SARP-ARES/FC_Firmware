#include "ctrldRogallo.h"
#include <cmath>
#include "GPS.h"

ctrldRogallo::ctrldRogallo() : gps(PA_2, PA_3), bmp(PB_7, PB_8, 0xEE), bno(PB_7, PB_8, 0x51), fc(PA_7, PA_6, PA_5, PA_4); {
    apogeeDetection = false;
    apogeeCounter = 0;
    fsm_mode = 0; // idle
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
    float targetHeading_rad = atan2(state.posEast, state.posNorth) + pi;
    float targetHeading_deg = targetHeading_rad * 180 / pi;
    return targetHeading_deg;
}

float ctrldRogallo::getThetaErr(){
    float thetaErr_deg = this->state.actHeading - this->state.targetHeading;
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
    // complementary kalman filter here
    // update actual and target heading
    gpsState state_gps = gps.getState();
    posLTP ltp = gps.getPosLTP();

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
    state.

    state.temp_c = bmp.getTemperatureC();
    state.pressure_pa = bmp.getPressurePa();



    

    
    this->state.actHeading = 0;
    this->state.targetHeading = 0;
}

