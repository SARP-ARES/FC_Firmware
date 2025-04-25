#include "ctrldRogallo.h"
#include <cmath>
#include "GPS.h"

ctrldRogallo::ctrldRogallo() : GPS gps(PA_2, PA_3), BMP280 bmp(PB_7, PB_8, 0xEE), BNO055 bno(PB_7, PB_8, 0x51) {}

// GPS::GPS(PinName rx_gps, PinName tx_gps) : serial(rx_gps, tx_gps) {}

filteredState ctrldRogallo::getState() const{
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



void ctrldRogallo::updateState(gpsState* stateGPS, bmpState* stateBMP, imuState* stateIMU){
    // complementary kalman filter here

    // if data (ex. gps lat lon) is the same, pass in a null pointer
    // update actual and target heading

    
    this->state.actHeading = 0;
    this->state.targetHeading = 0;
}

