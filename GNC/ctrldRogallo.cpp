#include "ctrldRogallo.h"
#include <cmath>
#include "EUSBSerial.h"
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "Mutex_I2C.h"
#include "PID.h"
#include <string>
#include "mbed.h"
#define MCPS_I2C_ADDR                   0x02 << 1 
#define DEG_LLA_TO_M_CONVERSION         111111
#define APOGEE_THRESHOLD_BUFFER         600
#define GROUNDED_THRESHOLD_BUFFER       100
#define ALPHA_ALT_START_PERCENT         0.05
#define SPIRAL_RADIUS                   10
#define PI                              3.1415926535
#define DEG_TO_RAD                      PI/180.0
#define BMP_I2C_ADDR                    0xEE
#define BNO_I2C_ADDR                    0x51

/**
 * @brief constructor that initializes the sensors and flash chip on the ARES flight computer.
 */ 
ctrldRogallo::ctrldRogallo(Mutex_I2C* i2c) 
    : gps(PA_2, PA_3), bmp(i2c, 0xEE), bno(i2c, 0x51), pid(1.0, 0.001, 0.1), i2c(i2c) {
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
 * @brief getter for the current FSM mode of the system
 * @returns FSM mode of the ctrldRogallo
 */ 
const ModeFSM ctrldRogallo::getMode() {
    return this->mode;
}


/**
 * @brief sets the seeking/landing target latitude & longitude
 * @param lat - target latitude
 * @param lon - target longitude
 * @todo WRITE TARGET TO FLASH CHIP
 */ 
void ctrldRogallo::setTarget(double lat, double lon) { 
    target_lat = lat; 
    target_lon = lon; 
    // TODO WRITE TARGET TO FLASH CHIP
}


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
    ScopedLock<Mutex> lock(this->state_mutex);
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


void ctrldRogallo::setPIDGains(float Kp, float Ki, float Kd) {
    this->pid.updateGains(Kp, Ki, Kd);
}

/**
 * @brief calculates target heading to point towards the origin in local tangent plane coords
 * @return target heading 
 */ 
float ctrldRogallo::getTargetHeading(){
    ScopedLock<Mutex> lock(this->state_mutex);
    float targetHeading_rad = atan2(state.pos_east_m, state.pos_north_m) + pi;
    float targetHeading_deg = targetHeading_rad * 180 / pi;
    return targetHeading_deg;
}

/**
 * @brief calculates heading error to feed into controller
 * @return heading error (current - target)
 */ 
float ctrldRogallo::getHeadingError(){
    ScopedLock<Mutex> lock(this->state_mutex);
    float thetaErr_deg = this->state.target_heading_deg - this->state.heading_deg;
    if (thetaErr_deg > 180){
        // if its greater than 180 deg, subtract 360 deg
        thetaErr_deg = thetaErr_deg - 360;
    }else if (thetaErr_deg < -180) {
        // if its less than -180 deg, add 360 deg
        thetaErr_deg = thetaErr_deg + 360;
    }
    return thetaErr_deg;
}

float ctrldRogallo::computeCtrl(float heading_error, float dt) {
    float delta_a_cmd = this->pid.compute(heading_error, dt);
    return delta_a_cmd;
}

void ctrldRogallo::resetFlightPacket() {
    ScopedLock<Mutex> lock(this->state_mutex);
    // Set all float fields to NAN
    state.timestamp_timer    = NAN;
    state.timestamp_gps      = NAN;
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
 * @brief updates the state of the system for control purposes and logging
 * @todo break into two functions. One that outputs a state packet of data
 *       and another that takes that flight packet as an arg and uses it to 
         update the internal state field of the CtrldRogallo object 
 */ 
void ctrldRogallo::updateFlightPacket(){
    GPSData gps_buf = gps.getData();
    BMPData bmp_buf = bmp.getData();
    IMUData bno_buf = bno.getData();

    // lock mutex for state field writes
    ScopedLock<Mutex> lock(this->state_mutex);

    state.timestamp_timer = getElapsedSeconds();
    state.fsm_mode = this->mode;

    // BMP 
    state.altitude_bmp_m = bmp_buf.altitude_m;
    state.temp_c = bmp_buf.temp_c;
    state.pressure_pa = bmp_buf.press_pa;

    // GPS
    state.timestamp_gps = gps_buf.utc;
    state.gps_fix = gps_buf.fix;
    state.heading_deg = gps_buf.heading;
    state.h_speed_m_s = gps_buf.gspeed;
    state.latitude_deg = gps_buf.lat;
    state.longitude_deg = gps_buf.lon;
    state.altitude_gps_m = gps_buf.alt;

    // state.altitude_m = getFuzedAlt(bmp_buf.altitude_m, gps_buf.alt); // shit don't work
    // state.altitude_m = bmp_buf.altitude_m; // TODO: replace with fuzedAlt

    updateHaversineCoords(); // TODO: restructure for lightweight mutex
    state.pos_east_m = haversineCoordEast;
    state.pos_north_m = haversineCoordNorth;
    state.distance_to_target_m = distanceToTarget;
    

    // BNO 
    state.bno_acc_x = bno_buf.acc_x;
    state.bno_acc_y = bno_buf.acc_y;
    state.bno_acc_z = bno_buf.acc_z;

    state.yaw_rate = bno_buf.gyro_x;
    state.pitch_rate = bno_buf.gyro_y;
    state.roll_rate = bno_buf.gyro_z;

    state.bno_mag_x = bno_buf.mag_x;
    state.bno_mag_y = bno_buf.mag_y;
    state.bno_mag_z = bno_buf.mag_z;

    state.bno_eul_x = bno_buf.eul_x;
    state.bno_eul_y = bno_buf.eul_y;
    state.bno_eul_z = bno_buf.eul_z;

    state.bno_lin_x = bno_buf.lin_x;
    state.bno_lin_y = bno_buf.lin_y;
    state.bno_lin_z = bno_buf.lin_z;

    state.bno_grav_x = bno_buf.grav_x;
    state.bno_grav_y = bno_buf.grav_y;
    state.bno_grav_z = bno_buf.grav_z;

    state.bno_quat_w = bno_buf.quat_w;
    state.bno_quat_x = bno_buf.quat_x;
    state.bno_quat_y = bno_buf.quat_y;
    state.bno_quat_z = bno_buf.quat_z;


    apogeeCounter += apogeeDetection(state.prevAlt, state.altitude_m); // checks if descending and above threshold

    if(apogeeCounter >= 200) {
        apogeeDetected = 1;
    }

    if(apogeeDetected == 1) {
        if(isWithinTarget())    mode = FSM_SPIRAL; // checks if ARES is within spiral target range mode = FSM_SPIRAL; 
        else                    mode = FSM_SEEKING;

        groundedCounter += groundedDetection(state.prevAlt, state.altitude_m); // checks if not moving and below threshold
        if(groundedCounter >= 15)    mode = FSM_GROUNDED;
    }


    state.apogee_counter = apogeeCounter;
    state.apogee_detected = apogeeDetected;
    state.groundedCounter = groundedCounter;

    state.prevAlt = state.altitude_m; 
    // mutex unlocks outside this scope
}

/** 
 *  @brief Sends control command over i2c to the MCPS 
 *  @param ctrl - asymetric deflection 
 *  @return 0 if success 1 if failure
 */
uint8_t ctrldRogallo::sendCtrl(float ctrl){

    uint8_t ack = i2c->write(MCPS_I2C_ADDR, reinterpret_cast<const char*>(&ctrl), sizeof(ctrl));
    wait_us(10);
    return ack; 
}

/**
 * @brief reads motor packet off of mcps 
 * @return packet read over i2c, true if success, false if fail
 */
bool ctrldRogallo::requestMotorPacket(){
    // grab motor packet over i2c
    uint8_t ack = i2c->read(MCPS_I2C_ADDR, rx_buf, sizeof(motorPacket)); 
    wait_us(10);

    if(ack != 0) return false; // failed
    
    // motor is a motorpacket object owned by ctrldRogallo
    memcpy(&motor, rx_buf, sizeof(motorPacket));
    return true; // success
}

/**
 * @brief updates BMP280 internal data struct (~50Hz)
 */ 
void ctrldRogallo::bmpUpdateLoop() {
    while (true) {
        bmp.update();
        ThisThread::sleep_for(20ms); // TODO: replace with timer
    }
}

/**
 * @brief updates GPS internal data struct
 */ 
void ctrldRogallo::gpsUpdateLoop(){
    while (true) {
        gps.bigUpdate(); // this takes 100-1000ms
        ThisThread::sleep_for(1ms); // avoid hammering just in case
    }
}

/**
 * @brief updates BNO055 internal data struct (~50Hz)
 */ 
void ctrldRogallo::imuUpdateLoop() {
    while (true) {
        bno.update();
        ThisThread::sleep_for(20ms); // TODO: replace with timer
    }
}

void ctrldRogallo::startThreadIMU() {
    // send event flag to start the thread
    this->thread_imu.start(callback(this, &ctrldRogallo::imuUpdateLoop));
}

void ctrldRogallo::startThreadBMP() {
    this->thread_bmp.start(callback(this, &ctrldRogallo::bmpUpdateLoop));
}

void ctrldRogallo::startThreadGPS(EUSBSerial* pc) {
    Thread thread_gps; // make new thread
    pc->printf("in startThreadGPS\n");
    this->thread_gps.start(callback(this, &ctrldRogallo::gpsUpdateLoop));
    pc->printf("started GPS thread!\n");
}


void ctrldRogallo::startAllSensorThreads(EUSBSerial* pc){
    pc->printf("now in 'startAllSensorThreads' func\n");
    startThreadGPS(pc);
    pc->printf("Starting IMU thread...\n");
    startThreadIMU();
    pc->printf("IMU thread started!\n");
    pc->printf("Starting BMP thread...\n");
    startThreadBMP();
    pc->printf("BMP thread started!\n");
}


void ctrldRogallo::killThreadIMU() {
    this->thread_imu.terminate();
}

void ctrldRogallo::killThreadBMP() {
    this->thread_bmp.terminate();
}

void ctrldRogallo::killThreadGPS() {
    this->thread_gps.terminate();
}

void ctrldRogallo::killAllSensorThreads(){
    killThreadGPS();
    killThreadIMU();
    killThreadBMP();
}

void ctrldRogallo::logDataLoop(){
    FlightPacket state_snapshot;
    Kernel::Clock::duration LOG_PERIOD = 1s; // default 1Hz
    while (true){
        auto start_time = flight_timer.elapsed_time();
        {   // take snapshot of current state w/ mutex
            ScopedLock<Mutex> lock(this->state_mutex);
            state_snapshot = this->state;
        }

        // write current state to flash chip & increment address
        flash_addr = flash_mem->writePacket(flash_addr, state_snapshot);
        
        // log at 10Hz while seeking or spiraling, 1Hz while idle, turn off once grounded
        switch (this->mode) {
            case FSM_IDLE:      LOG_PERIOD = 1s; break; // 1Hz
            case FSM_SEEKING:   LOG_PERIOD = 100ms; break; // 10Hz
            case FSM_SPIRAL:    LOG_PERIOD = 100ms; break; // 10Hz
            case FSM_GROUNDED:  stopLogging(); return;
        }   
        auto end_time = flight_timer.elapsed_time();
        auto compute_time = end_time - start_time;
        // wait until next period to log next packet
        if (compute_time < LOG_PERIOD) {
            ThisThread::sleep_for(
                chrono::duration_cast<Kernel::Clock::duration>(
                    LOG_PERIOD - compute_time
                )
            );
        } // else log the next packet ASAP
    }
}

void ctrldRogallo::startLogging(flash* flash_mem, EUSBSerial* pc) {
    Thread thread_logging; // make new thread
    pc->printf("made it into startLogging\n");
    // flash_mem and flash_addr are initalized in the main program and passed in here
    this->flash_mem = flash_mem;
    pc->printf("about to getNumPacketsWritten\n");
    uint32_t previous_num_packets = flash_mem->getNumPacketsWritten();
    pc->printf("made it past getNumPacketsWritten... previous packets: %d\n", previous_num_packets);
    flash_addr = previous_num_packets * 256; // start logging at next empty page
    pc->printf("made it past flash mem and addr assignments\n");
    pc->printf("flash_mem=%p, flash_addr=%d\n", flash_mem, flash_addr);
    flight_timer.start(); // start timer once logging begins
    thread_logging.start(callback(this, &ctrldRogallo::logDataLoop));
}

void ctrldRogallo::stopLogging(){
    this->thread_logging.terminate();
}

void ctrldRogallo::stopAllThreads(){
    this->stopLogging();
    this->killAllSensorThreads();
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
    ScopedLock<Mutex> lock(this->state_mutex);
    groundedThreshold = state.altitude_m + GROUNDED_THRESHOLD_BUFFER;
    apogeeThreshold = state.altitude_m + APOGEE_THRESHOLD_BUFFER; 
}

void ctrldRogallo::printCompactState(EUSBSerial* pc) {
    ScopedLock<Mutex> lock(this->state_mutex);
    pc->printf("Timer:\t\t\t\t\t%f s\n", state.timestamp_timer);
    pc->printf("Lat (deg), Lon (deg), Alt (m):\t\t%f, %f, %.3f\n", 
                state.latitude_deg, state.longitude_deg, state.altitude_m);
    pc->printf("Pos North (m), Pos East (m):\t\t%.2f, %.2f\n", 
                state.pos_north_m, state.pos_east_m);
    pc->printf("(Heading, deg) Current, Desired, Error:\t%.1f, %.1f, %.1f\n", 
                state.heading_deg, state.target_heading_deg, state.heading_error_deg);
    pc->printf("FC CMD:\t\t\t\t\t%.1f\n", state.fc_cmd); // TODO: implement PID 
    pc->printf("Distance to Target (m):\t\t\t%.2f\n", state.distance_to_target_m);
    pc->printf("FSM mode:\t\t\t\t%d\n", state.fsm_mode);
    pc->printf("Apogee Counter:\t\t\t\t%d\n", state.apogee_counter);
    pc->printf("Apogee Detected:\t\t\t%d\n", state.apogee_detected);
    pc->printf("Grounded Counter:\t\t\t%d \n", state.groundedCounter);
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

