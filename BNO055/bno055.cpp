#include "mbed.h"
#include "BNO055.h"
#include "bno055_const.h"
#include <map>

void wait (int time_ms){
    ThisThread::sleep_for(time_ms);
}

/**
 * @brief Constructor that uses an external I2C object (not owned by this class).
 * @param i2c Pointer to an already-initialized I2C object
 */
BNO055::BNO055(Mutex_I2C* i2c, char addr) {
    BNO055::i2c = i2c;
    BNO055::addr = addr;
}

/**
 * @brief Destructor. Deletes internally-owned I2C object if we created it.
 */
BNO055::~BNO055() {
    if (owned) {
        delete i2c;
    }
}

/**
 * @brief Dummy function (no operation). Could be used to ensure link or for debug.
 */
void BNO055::dummy(){
    // No operation
}

/**
 * @brief Checks the system calibration status.
 * @return 0 if system is fully calibrated, 1 otherwise
 */
int BNO055::getSysCalib(){
    char stat;
    i2c->readData(addr, BNO055_CALIB_STAT, &stat, 1);
    // Bits 6-7 hold the system calibration status (3 = fully calibrated)
    if (((stat >> 6) & 0x03) != 3){
        return 1;
    }
    return 0;
}

/**
 * @brief Checks the gyroscope calibration status.
 * @return 0 if fully calibrated, 1 otherwise
 */
int BNO055::getGyrCalib(){
    char stat;
    i2c->readData(addr, BNO055_CALIB_STAT, &stat, 1);
    // Bits 4-5 hold the gyroscope calibration status (3 = fully calibrated)
    if (((stat >> 4) & 0x03) != 3){
        return 1;
    }
    return 0;
}

/**
 * @brief Checks the accelerometer calibration status.
 * @return 0 if fully calibrated, 1 otherwise
 */
int BNO055::getAccCalib(){
    char stat;
    i2c->readData(addr,BNO055_CALIB_STAT, &stat, 1);
    // Bits 2-3 hold the accelerometer calibration status (3 = fully calibrated)
    if (((stat >> 2) & 0x03) != 3){
        return 1;
    }
    return 0;
}

/**
 * @brief Checks the magnetometer calibration status.
 * @return 0 if fully calibrated, 1 otherwise
 */
int BNO055::getMagCalib(){
    char stat;
    i2c->readData(addr, BNO055_CALIB_STAT, &stat, 1);
    // Bits 0-1 hold the magnetometer calibration status (3 = fully calibrated)
    if (((stat) & 0x03) != 3){
        return 1;
    }
    return 0;
}

/**
 * @brief Sets the power mode of the BNO055.
 * @param mode Power mode to set (Normal, LowPower, Suspend)
 */
void BNO055::setPWR(PWRMode mode) {
    char modeData;
    switch(mode) {
        case PWRMode::Normal:   modeData = 0x00; break;
        case PWRMode::LowPower: modeData = 0x01; break;
        case PWRMode::Suspend:  modeData = 0x02; break;
    }
    i2c->writeData(addr, BNO055_PWR_MODE, modeData);
}

/**
 * @brief Reads the current operating mode (OPR_MODE) from the BNO055.
 * @return The raw OPR_MODE register value
 */
char BNO055::getOPMode() {
    setPage(0);
    char mode;
    i2c->readData(addr, BNO055_OPR_MODE, &mode, 1);
    return mode;
}

/**
 * @brief Sets the operating mode (OPR_MODE) of the BNO055.
 *        The sensor must be in CONFIG mode before certain configurations.
 * @param mode The operating mode to set (e.g., CONFIG, NDOF, IMU, etc.)
 */
void BNO055::setOPMode(char mode) {
    setPage(0);
    i2c->writeData(addr, BNO055_OPR_MODE, mode);

    // According to Bosch datasheet, wait times vary after setting OPR_MODE:
    // 19ms if switching to CONFIG mode, 7ms otherwise.
    if (mode == BNO055_OPERATION_MODE_CONFIG) {
        wait(19);
    } else {
        wait(7);
    }
}

/**
 * @brief Configures the accelerometer settings.
 * @param GRange Bits representing g-range
 * @param Bandwidth Bits representing bandwidth
 * @param OPMode Bits for operating mode
 */
void BNO055::setACC(char GRange, char Bandwidth, char OPMode) {
    setPage(0);
    char config = GRange | Bandwidth | OPMode;
    i2c->writeData(addr, BNO055_ACC_CONFIG, config);
    wait(20);
}

/**
 * @brief Configures the gyroscope settings.
 * @param Range Bits for angular rate range
 * @param Bandwidth Bits for filter bandwidth
 * @param OPMode Bits for gyroscope power/operation mode
 */
void BNO055::setGYR(char Range, char Bandwidth, char OPMode) {
    setPage(0);
    char config0 = Range | Bandwidth;
    char config1 = OPMode;
    i2c->writeData(addr, BNO055_GYRO_CONFIG_0, config0);
    wait(20);
    i2c->writeData(addr, BNO055_GYRO_CONFIG_1, config1);
    wait(20);
}

/**
 * @brief Configures the magnetometer settings.
 * @param Rate Output data rate
 * @param OPMode Operating mode (e.g., regular, enhanced)
 * @param Power Power mode for the magnetometer
 */
void BNO055::setMAG(char Rate, char OPMode, char Power) {
    setPage(0);
    char config = Rate | OPMode | Power;
    i2c->writeData(addr, BNO055_MAG_CONFIG, config);
    wait(20);
}

/**
 * @brief Sets the page register for accessing different sets of registers.
 * @param page Page number (0 or 1)
 */
void BNO055::setPage(uint8_t page) {
    char pageChar = static_cast<char>(page);
    i2c->writeData(addr, BNO055_PAGE_ID, pageChar);
}

/**
 * @brief Sets the clock source of the BNO055 (internal or external).
 *        Wait is required for stable operation when switching clocks.
 * @param external true = use external clock, false = use internal clock
 */
void BNO055::setCLK(bool external) {
    setPage(0);
    char tmp = 0x00;
    i2c->readData(addr,BNO055_SYS_TRIGGER, &tmp, 1);
    tmp |= external ? 0x80 : 0x00;
    i2c->writeData(addr,BNO055_SYS_TRIGGER, tmp);
    wait(700);
}

/**
 * @brief Configures units for accelerometer, angular rate, euler, temperature, and selects fusion or raw.
 * @param acc true = m/s^2, false = mg
 * @param angular true = degrees/s, false = rad/s
 * @param euler true = degrees, false = radians
 * @param temp true = Fahrenheit, false = Celsius
 * @param fusion true = oriented in fusion ops, false = raw
 */
void BNO055::setUnit(bool acc, bool angular, bool euler, bool temp, bool fusion) {
    setPage(0);
    char config = 0x00;
    config |= acc;
    config |= (angular << 1);
    config |= (euler << 2);
    config |= (temp << 4);
    config |= (fusion << 7);
    i2c->writeData(addr,BNO055_UNIT_SEL, config);
    wait(20);
}

/**
 * @brief Resets the BNO055 via software reset (SYS_TRIGGER register).
 *        Sensor will re-initialize, requiring time to reboot.
 */
void BNO055::reset() {
    char resetVal = 0x20;
    i2c->writeData(addr,BNO055_SYS_TRIGGER, resetVal);
    wait(700);
}

/**
 * @brief Resets the BNO055 via an external pin (nReset line).
 *        This requires hardware connection of rst to the BNO055 reset pin.
 *        Define a global rst pin in main. 
 */
void BNO055::nReset() {
    //rst = 0;
    wait(500);
    //rst = 1;
    wait(500);
}

/**
 * @brief Set custom axis mapping (X, Y, Z). The user can also set negative sign bits if needed.
 * @param newX Axis assignment for X
 * @param newY Axis assignment for Y
 * @param newZ Axis assignment for Z
 * @param xNeg Whether X axis is reversed
 * @param yNeg Whether Y axis is reversed
 * @param zNeg Whether Z axis is reversed
 */
void BNO055::setAxes(Axes newX, Axes newY, Axes newZ, bool xNeg, bool yNeg, bool zNeg) {
    char axes = getAxes(newX, newY, newZ);
    i2c->writeData(addr,BNO055_AXIS_MAP_CONFIG, axes);
    wait(20);
    char sign = getAxesSign(xNeg, yNeg, zNeg);
    i2c->writeData(addr, BNO055_AXIS_MAP_SIGN, sign);
    wait(20);
    // If sign inversion is desired, call setAxesSign(...) or integrate it here
}

/**
 * @brief Helper function to calculate axis remap configuration byte from Axes enum values.
 * @param newX, newY, newZ Desired axis mapping
 * @return Single byte that encodes axis remap for the BNO055
 */
char BNO055::getAxes(Axes newX, Axes newY, Axes newZ) {
    static std::map<Axes, char> axesMap = {
        {Axes::X, 0x00},
        {Axes::Y, 0x01},
        {Axes::Z, 0x02}
    };
    char axes = 0x00;
    axes |= axesMap[newX];
    axes |= (axesMap[newY] << 2);
    axes |= (axesMap[newZ] << 4);
    return axes;
}

/**
 * @brief Helper function to encode sign inversion for each axis.
 * @param xNeg, yNeg, zNeg Whether to invert each axis
 * @return The sign configuration byte
 */
char BNO055::getAxesSign(bool xNeg, bool yNeg, bool zNeg) {
    char sign = 0x00;
    // NOTE: According to BNO055 datasheet, the bits for each axis sign 
    // are placed in bits [0,1,2] respectively, but might also differ 
    // if you need xNeg in bit 0, yNeg in bit 1, zNeg in bit 2, etc.
    sign |= (xNeg ? 0x01 : 0x00);
    sign |= (yNeg ? 0x02 : 0x00);
    sign |= (zNeg ? 0x04 : 0x00);
    return sign;
}

/**
 * @brief Sets the accelerometer offset registers.
 * @param offsetX, offsetY, offsetZ 16-bit offset values
 */
void BNO055::setACCOffset(uint16_t offsetX, uint16_t offsetY, uint16_t offsetZ) {
    char lsbX  = (char)(offsetX & 0xFF); 
    char msbX = (char)((offsetX >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_ACC_OFFSET_X_LSB, lsbX);
    i2c->writeData(addr,BNO055_ACC_OFFSET_X_MSB, msbX);

    char lsbY  = (char)(offsetY & 0xFF); 
    char msbY = (char)((offsetY >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_ACC_OFFSET_Y_LSB, lsbY);
    i2c->writeData(addr,BNO055_ACC_OFFSET_Y_MSB, msbY);

    char lsbZ  = (char)(offsetZ & 0xFF); 
    char msbZ = (char)((offsetZ >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_ACC_OFFSET_Z_LSB, lsbZ);
    i2c->writeData(addr,BNO055_ACC_OFFSET_Z_MSB, msbZ);
}

/**
 * @brief Sets the magnetometer offset registers.
 */
void BNO055::setMAGOffset(uint16_t offsetX, uint16_t offsetY, uint16_t offsetZ) {
    char lsbX  = (char)(offsetX & 0xFF); 
    char msbX = (char)((offsetX >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_MAG_OFFSET_X_LSB, lsbX);
    i2c->writeData(addr,BNO055_MAG_OFFSET_X_MSB, msbX);

    char lsbY  = (char)(offsetY & 0xFF); 
    char msbY = (char)((offsetY >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_MAG_OFFSET_Y_LSB, lsbY);
    i2c->writeData(addr,BNO055_MAG_OFFSET_Y_MSB, msbY);

    char lsbZ  = (char)(offsetZ & 0xFF); 
    char msbZ = (char)((offsetZ >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_MAG_OFFSET_Z_LSB, lsbZ);
    i2c->writeData(addr,BNO055_MAG_OFFSET_Z_MSB, msbZ); 
}

/**
 * @brief Sets the gyroscope offset registers.
 */
void BNO055::setGYROffset(uint16_t offsetX, uint16_t offsetY, uint16_t offsetZ) {
    char lsbX  = (char)(offsetX & 0xFF); 
    char msbX = (char)((offsetX >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_GYR_OFFSET_X_LSB, lsbX);
    i2c->writeData(addr,BNO055_GYR_OFFSET_X_MSB, msbX);

    char lsbY  = (char)(offsetY & 0xFF); 
    char msbY = (char)((offsetY >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_GYR_OFFSET_Y_LSB, lsbY);
    i2c->writeData(addr,BNO055_GYR_OFFSET_Y_MSB, msbY);

    char lsbZ  = (char)(offsetZ & 0xFF); 
    char msbZ = (char)((offsetZ >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_GYR_OFFSET_Z_LSB, lsbZ);
    i2c->writeData(addr,BNO055_GYR_OFFSET_Z_MSB, msbZ);
}

/**
 * @brief Sets the accelerometer and magnetometer radius registers, used for calibration.
 */
void BNO055::setRadius(uint16_t accRadius, uint16_t magRadius) {
    char lsbAcc  = (char)(accRadius & 0xFF); 
    char msbAcc = (char)((accRadius >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_ACC_RADIUS_LSB, lsbAcc);
    i2c->writeData(addr,BNO055_ACC_RADIUS_MSB, msbAcc);

    char lsbMag  = (char)(magRadius & 0xFF); 
    char msbMag = (char)((magRadius >> 8) & 0xFF);
    i2c->writeData(addr,BNO055_MAG_RADIUS_LSB, lsbMag);
    i2c->writeData(addr,BNO055_MAG_RADIUS_MSB, msbMag);
}

/**
 * @brief Reads the accelerometer calibration radius.
 * @return 16-bit radius value
 */
uint16_t BNO055::getAccRadius(){
    char lsb, msb;
    i2c->readData(addr, BNO055_ACC_RADIUS_LSB, &lsb, 1);
    i2c->readData(addr, BNO055_ACC_RADIUS_MSB, &msb, 1);
    uint16_t radius = static_cast<uint16_t>((msb << 8) | lsb);
    return radius;
}

/**
 * @brief Reads the magnetometer calibration radius.
 * @return 16-bit radius value
 */
uint16_t BNO055::getMagRadius(){
    char lsb, msb;
    i2c->readData(addr,BNO055_MAG_RADIUS_LSB, &lsb, 1);
    i2c->readData(addr,BNO055_MAG_RADIUS_MSB, &msb, 1);
    uint16_t radius = static_cast<uint16_t>((msb << 8) | lsb);
    return radius;  
}

/**
 * @brief Retrieves the magnetometer offsets from the BNO055 registers.
 * @return offset struct with X, Y, Z 16-bit offsets
 */
offset BNO055::getMagOffset(){
    char lsbX, msbX;
    i2c->readData(addr,BNO055_MAG_OFFSET_X_LSB, &lsbX, 1);
    i2c->readData(addr,BNO055_MAG_OFFSET_X_MSB, &msbX, 1);
    uint16_t offsetX = static_cast<uint16_t>((msbX << 8) | lsbX);

    char lsbY, msbY;
    i2c->readData(addr,BNO055_MAG_OFFSET_Y_LSB, &lsbY, 1);
    i2c->readData(addr,BNO055_MAG_OFFSET_Y_MSB, &msbY, 1);
    uint16_t offsetY = static_cast<uint16_t>((msbY << 8) | lsbY);

    char lsbZ, msbZ;
    i2c->readData(addr,BNO055_MAG_OFFSET_Z_LSB, &lsbZ, 1);
    i2c->readData(addr,BNO055_MAG_OFFSET_Z_MSB, &msbZ, 1);
    uint16_t offsetZ = static_cast<uint16_t>((msbZ << 8) | lsbZ);
    
    return offset{offsetX, offsetY, offsetZ};
}

/**
 * @brief Retrieves the accelerometer offsets.
 */
offset BNO055::getAccOffset(){
    char lsbX, msbX;
    i2c->readData(addr,BNO055_ACC_OFFSET_X_LSB, &lsbX, 1);
    i2c->readData(addr,BNO055_ACC_OFFSET_X_MSB, &msbX, 1);
    uint16_t offsetX = static_cast<uint16_t>((msbX << 8) | lsbX);

    char lsbY, msbY;
    i2c->readData(addr,BNO055_ACC_OFFSET_Y_LSB, &lsbY, 1);
    i2c->readData(addr,BNO055_ACC_OFFSET_Y_MSB, &msbY, 1);
    uint16_t offsetY = static_cast<uint16_t>((msbY << 8) | lsbY);

    char lsbZ, msbZ;
    i2c->readData(addr,BNO055_ACC_OFFSET_Z_LSB, &lsbZ, 1);
    i2c->readData(addr,BNO055_ACC_OFFSET_Z_MSB, &msbZ, 1);
    uint16_t offsetZ = static_cast<uint16_t>((msbZ << 8) | lsbZ);
    
    return offset{offsetX, offsetY, offsetZ};
}

/**
 * @brief Retrieves the gyroscope offsets.
 */
offset BNO055::getGyrOffset(){
    char lsbX, msbX;
    i2c->readData(addr,BNO055_GYR_OFFSET_X_LSB, &lsbX, 1);
    i2c->readData(addr,BNO055_GYR_OFFSET_X_MSB, &msbX, 1);
    uint16_t offsetX = static_cast<uint16_t>((msbX << 8) | lsbX);

    char lsbY, msbY;
    i2c->readData(addr,BNO055_GYR_OFFSET_Y_LSB, &lsbY, 1);
    i2c->readData(addr,BNO055_GYR_OFFSET_Y_MSB, &msbY, 1);
    uint16_t offsetY = static_cast<uint16_t>((msbY << 8) | lsbY);

    char lsbZ, msbZ;
    i2c->readData(addr,BNO055_GYR_OFFSET_Z_LSB, &lsbZ, 1);
    i2c->readData(addr,BNO055_GYR_OFFSET_Z_MSB, &msbZ, 1);
    uint16_t offsetZ = static_cast<uint16_t>((msbZ << 8) | lsbZ);
    
    return offset{offsetX, offsetY, offsetZ};
}

/**
 * @brief Reads the current system error code.
 * @return System error code (see datasheet)
 */
char BNO055::get_SysErr() {
    setPage(0);
    char err = 0x00;
    i2c->readData(addr, BNO055_SYS_ERR, &err, 1);
    return err;
}

/**
 * @brief Reads the current system status code.
 * @return System status code (see datasheet for meaning)
 */
char BNO055::get_SysStatus() {
    setPage(0);
    char status = 0x00;
    i2c->readData(addr,BNO055_SYS_STATUS, &status, 1);
    return status;
}

/**
 * @brief Initiates a self-test by setting the SELF_TEST bit in SYS_TRIGGER.
 */
void BNO055::runSelfTest() {
    char set;
    i2c->readData(addr, BNO055_SYS_TRIGGER, &set, 1);
    set |= 0x01; // Set SELF_TEST bit
    i2c->writeData(addr, BNO055_SYS_TRIGGER, set);
    wait(20);
}

/**
 * @brief Reads the self-test result register and interprets it.
 * @return An enum indicating if each sub-test passed or if an error occurred.
 */
BNO055Result BNO055::readSelfTest() {
    setPage(0);
    char res = 0;
    i2c->readData(addr,BNO055_ST_RESULT, &res, 1);

    // Bits in ST_RESULT:
    // [3] = System test, [2] = Gyro, [1] = Acc, [0] = Mag
    char sys = (res >> 3) & 0x01;
    char gyr = (res >> 2) & 0x01;
    char acc = (res >> 1) & 0x01;
    char mag = res & 0x01;

    if (sys != 1) {
        return BNO055Result::SysErr;
    } else if (gyr != 1) {
        return BNO055Result::GyrErr;
    } else if (acc != 1) {
        return BNO055Result::AccErr;
    } else if (mag != 1) {
        return BNO055Result::MagErr;
    } else {
        return BNO055Result::Ok;
    }
}

/**
 * @brief Basic setup function for the BNO055:
 *        - Resets the device
 *        - Sets page to 0
 *        - Configures to NDOF (sensor-fusion) operating mode
 * @return BNO055Result::Ok if successful
 */
BNO055Result BNO055::setup() {
    reset();
    setPage(0);
    setOPMode(BNO055_OPERATION_MODE_NDOF);
    // Add any other configurations needed (units, axis remap, calibration, etc.)

    return BNO055Result::Ok;
}

/**
 * @brief Puts the sensor into the Suspend power mode.
 * @return BNO055Result::Ok if successful
 */
BNO055Result BNO055::stop() {
    setPWR(PWRMode::Suspend);
    return BNO055Result::Ok;
}

/**
 * @brief Generic function to read a vector from the BNO055, such as
 *        ACCELEROMETER, GYROSCOPE, EULER, QUATERNION, etc.
 * @param vec One of the BNO055_VECTOR_* constants
 * @return A struct containing the x, y, z, (and w if quaternion) data.
 */
bno055_vector_t BNO055::bno055_getVector(char vec) {
    setPage(0);

    // Quaternion reads 8 bytes, others read 6 bytes
    char buffer[8] = {0};
    if (vec == BNO055_VECTOR_QUATERNION) {
        i2c->readData(addr,vec, buffer, 8);
    } else {
        i2c->readData(addr,vec, buffer, 6);
    }

    // Determine scaling based on vector type
    double scale = 1.0;
    if (vec == BNO055_VECTOR_MAGNETOMETER) {
        scale = magScale;         // Typically 1 LSB = 1 µT
    } else if (vec == BNO055_VECTOR_ACCELEROMETER ||
               vec == BNO055_VECTOR_LINEARACCEL  ||
               vec == BNO055_VECTOR_GRAVITY) {
        scale = accelScale;       // Typically 1 LSB = 1 mg or 1 m/s^2 depending on unit settings
    } else if (vec == BNO055_VECTOR_GYROSCOPE) {
        scale = angularRateScale; // Typically 1 LSB = 1 dps or rad/s
    } else if (vec == BNO055_VECTOR_EULER) {
        scale = eulerScale;       // Typically 1 LSB = 1 degree
    } else if (vec == BNO055_VECTOR_QUATERNION) {
        scale = quaScale;         // Typically 1 LSB = 1/16384
    }

    bno055_vector_t xyz{};
    xyz.w = 0.0;

    // If it's a quaternion, parse 4 int16_t values (w, x, y, z).
    if (vec == BNO055_VECTOR_QUATERNION) {
        xyz.w = static_cast<int16_t>((buffer[1] << 8) | buffer[0]) / scale;
        xyz.x = static_cast<int16_t>((buffer[3] << 8) | buffer[2]) / scale;
        xyz.y = static_cast<int16_t>((buffer[5] << 8) | buffer[4]) / scale;
        xyz.z = static_cast<int16_t>((buffer[7] << 8) | buffer[6]) / scale;
    } else {
        xyz.x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]) / scale;
        xyz.y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]) / scale;
        xyz.z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]) / scale;
    }

    return xyz;
}

/**
 * @brief Reads the current temperature from the BNO055 sensor.
 *        Units depend on setUnit() configuration (Celsius or Fahrenheit).
 * @return The temperature as a float
 */
float BNO055::getTemperature(){
    char temp;
    i2c->readData(addr,BNO055_TEMP, &temp, 1);
    return temp;
}

/**
 * @brief Returns a vector with accelerometer data.
 */
bno055_vector_t BNO055::getAccelerometer() {
    return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}

/**
 * @brief Returns a vector with magnetometer data.
 */
bno055_vector_t BNO055::getMagnetometer() {
    return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}

/**
 * @brief Returns a vector with gyroscope data.
 */
bno055_vector_t BNO055::getGyroscope() {
    return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}

/**
 * @brief Returns a vector with Euler angles (heading, roll, pitch).
 */
bno055_vector_t BNO055::getEuler() {
    return bno055_getVector(BNO055_VECTOR_EULER);
}

/**
 * @brief Returns a vector representing linear acceleration (acceleration minus gravity).
 */
bno055_vector_t BNO055::getLinearAccel() {
    return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}

/**
 * @brief Returns a vector representing the direction of gravity.
 */
bno055_vector_t BNO055::getGravity() {
    return bno055_getVector(BNO055_VECTOR_GRAVITY);
}

/**
 * @brief Returns the quaternion representation from the BNO055 sensor fusion.
 */
bno055_vector_t BNO055::getQuaternion() {
    return bno055_getVector(BNO055_VECTOR_QUATERNION);  
}

/**
 * @brief Updates all IMU data in member struct
 */
void BNO055::update() {
    bno055_vector_t acc = getAccelerometer();
    bno055_vector_t gyro = getGyroscope();
    bno055_vector_t mag = getMagnetometer();
    bno055_vector_t eul = getEuler();
    bno055_vector_t lin = getLinearAccel();
    bno055_vector_t grav = getGravity();
    bno055_vector_t quat = getQuaternion();

    ScopedLock<Mutex> lock(this->mutex);
    data.acc_x = acc.x;
    data.acc_y = acc.y;
    data.acc_z = acc.z;
    data.gyro_x = gyro.x;
    data.gyro_y = gyro.y;
    data.gyro_z = gyro.z;
    data.mag_x = mag.x;
    data.mag_y = mag.y;
    data.mag_z = mag.z;
    data.eul_x = eul.x;
    data.eul_y = eul.y;
    data.eul_z = eul.z;
    data.lin_x = lin.x;
    data.lin_y = lin.y;
    data.lin_z = lin.z;
    data.grav_x = grav.x;
    data.grav_y = grav.y;
    data.grav_z = grav.z;
    data.quat_w = quat.w;
    data.quat_x = quat.x;
    data.quat_y = quat.y;
    data.quat_z = quat.z;
}


/**
 * @brief Returns all IMU data
 * @returns IMUData struct of all data
 */
IMUData BNO055::getData() const{
    ScopedLock<Mutex> lock(this->mutex);
    return data;
}