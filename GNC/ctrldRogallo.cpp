#include "ctrldRogallo.h"


filteredState ctrldRogallo::getState() const{
    return this->state;
}

float ctrldRogallo::getThetaErr(){
    float current = this->state.actHeading;
    float target = this->state.targetHeading;
    float thetaErr = target - current;
    return thetaErr;
}

float ctrldRogallo::getTargetHeading(){
    
    
    return targetHeading;
}

void ctrldRogallo::updateState(gpsState stateGPS, bmpState stateBMP, imuState stateIMU){
    // complementary kalman filter here

    // update actual and target heading
    this->state.actHeading = 0;
    this->state.targetHeading = 0;
}
