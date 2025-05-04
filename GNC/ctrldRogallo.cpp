#include "ctrldRogallo.h"
#include <cmath>
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"


/**
 * @brief constructor that initializes the sensors and flash chip on the ARES flight computer.
 */ 
ctrldRogallo::ctrldRogallo() 
    : gps(PA_2, PA_3), bmp(PB_7, PB_8, 0xEE), bno(PB_7, PB_8, 0x51), fc_mcps(PB_3, PB_10, 0x32, false){
    bmp.start();
    bno.setup();
    resetFlightPacket();
    apogeeDetected = 0; // false
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


MotorData ctrldRogallo::getMotorData(){
    MotorData motor_state;
    fc_mcps.read(reinterpret_cast<char*>(&motor_state), sizeof(MotorData));

    return motor_state;
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
    state.pos_up_m           = NAN;
    state.temp_c             = NAN;
    state.pressure_pa        = NAN;
    state.delta1_deg         = NAN;
    state.delta1_m           = NAN;
    state.delta2_deg         = NAN;
    state.delta2_m           = NAN;
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
    state.gps_antenna_status = 0xFF;
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
    double prevAlt = bmp_state.altitude_m;
    
    bmp.updateValues();
    int success = gps.bigUpdate(); 

    gpsState gps_state = gps.getState();
    bmp_state = bmp.getState(); 
    MotorData motor_state = getMotorData();
    posLTP ltp = gps.getPosLTP();

    // MCPS
    state.delta1_deg = motor_state.delta1_deg;
    state.delta2_deg = motor_state.delta2_deg;
    state.delta1_m = state.delta1_deg / 360 * SPOOL_CIRC;
    state.delta2_m = state.delta2_deg / 360 * SPOOL_CIRC;
    state.pwm_motor1 = motor_state.pwm_motor1;
    state.pwm_motor2 = motor_state.pwm_motor2;


    // GPS 
    state.timestamp_utc = gps_state.utc;
    state.fsm_mode = this->mode;
    state.gps_fix = gps_state.fix;
    state.gps_antenna_status = gps_state.antenna_status;
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
    state.apogee_counter = apogeeCounter;
    state.apogee_detected = apogeeDetected;


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


    state.compass_heading = getCompassDirection();


    apogeeCounter += apogeeDetection(prevAlt, state.altitude_m);
    if(apogeeCounter >= 15){
        apogeeDetected = 1; // true
        // trigger seeking mode
        mode = FSM_SEEKING; // 1
    }


    strncpy(state.flight_id, "ARES-01\0", sizeof(state.flight_id));

    
}

/**
 * @Brief fuzes the altitude of the GPS and BMP reading using a complimentary filter
 * @return - the fuzed altitude of the two sensors
 */ 
float ctrldRogallo::getFuzedAlt(float alt1, float alt2){
    float fuzedAlt = NAN; 
    // check for NANs (they will not equal themselves)
    if (alt1 == alt1 && alt2 == alt2){
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
 * @breif - detects if rocket has reached apogee based upon current velocity * (-1.5 m/s constitutes as apogee)
 * @param prevAlt - previous altitude 
 * @param currAlt - current altitude
 * @return 0 if non apogee 1 if apogee
 */ 
uint32_t ctrldRogallo::apogeeDetection(double prevAlt, double currAlt){
    double interval = 1; // seconds
    double apogeeVelo = -1.2; // m/s
    double velo = (currAlt - prevAlt)/interval;
    if(velo <= apogeeVelo && currAlt >= 600) { // 600m threshold altitude
        return 1;
    }
    return 0; 
}


char* ctrldRogallo::getCompassDirection(){
    float heading = state.heading_deg;
    char* direction = new char[3];
    while(heading < 0){
        heading += 360; 
    }
    while(heading > 360){
        heading -= 360;
    } 
    if(heading > 360-22.5 || heading <= 22.5 ) {
        direction[0] = 'N';
        direction[1] = '\0';
        return direction;
    } 
    if(heading < 67.5){
        direction[0] = 'N';
        direction[1] = 'E';
        direction[2] = '\0';
        return direction;
    }
    if(heading < 117.5){
        direction[0] = 'E';
        direction[1] = '\0';
        return "E";
    }
    if(heading < 167.5 ){
        direction[0] = 'S';
        direction[1] = 'E';
        direction[2] = '\0';
        return "SE";
    }
    if(heading < 217.5){
        direction[0] = 'S';
        direction[1] = '\0';
        return "S";
    }
    if(heading < 267.5){
        direction[0] = 'S';
        direction[1] = 'W';
        direction[2] = '\0';
        return "SW";
    }
    if(heading < 317.5){
        direction[0] = 'W';
        direction[1] = '\0';
        return "W";
    }
    direction[0] = 'N';
    direction[1] = 'W';
    direction[2] = '\0';
    return direction;
    return "NW";
}   


