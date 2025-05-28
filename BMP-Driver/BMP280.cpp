#include"mbed.h"
#include"BMP280.h" 
#include"BMP280_const.h" 
#include<map> 
#include<iostream>
#include<cmath>
using namespace std;

/** Constructor 
 * @param Pin address of the SDA Pin on the processor 
 * @param Pin address of the SCL Pin on the processor    
 * @param The I2C address for the BMP280
 */
BMP280::BMP280(PinName SDA, PinName SCL, char addr){ 
    owned = true; 
    BMP280::i2c = new I2C(SDA,SCL);
    BMP280::addr = addr; 
}

/** Destructor
 * @brief Deletes I2C if created in object
 */ 
BMP280::~BMP280() {
    if(owned){
        delete i2c; 
    }
}

/**
 * @brief Reads a chunk of data from the BNO055 over I2C.
 * @param regaddr The register address to read from
 * @param data Pointer to a buffer for storing data
 * @param len Number of bytes to read
 * @return 0 on success, non-zero on failure
 */
int BMP280::readData(char regaddr, char* data, uint8_t len) {
    i2c->write(addr, &regaddr, 1);
    return i2c->read(addr, data, len);
}

/** 
 * @brief Writes data to a BNO055 register over I2C.
 * @param regaddr The register address to write to
 * @param data The value to be written
 * @return 0 on success, non-zero on failure
 */
int BMP280::writeData(char regaddr, char data) {
    char buffer[2];
    buffer[0] = regaddr;
    buffer[1] = data;
    return i2c->write(addr, buffer, 2);
}

// Temperature 2x, Pressure 16x -> 0b11101011
// Temperature 1x, Pressure 1x -> 0b00100111
/** 
 * @brief Starts the BMP280 in standard operating mode using given sampling rate
 * @return - state of write 0 if written 1 if error 
 */
int BMP280::start(){
    int result = writeData(BMP280_CTRL_MEAS, 0b11101011);
    // 11 = normal mode
    return result;
}

/**
 * @brief Sleeps the BMP280, will not collect data in this mode
 * @return - state of write 0 if written 1 if error 
 */
int BMP280::sleep(){
    int result = writeData(BMP280_CTRL_MEAS, 0b11101000);
    // 00 = sleep mode
    return result;
}

/**
 * @brief Retrieves and organizes temperature data - stores the values in the BMP_280VALUES struct
 * @return calulated temperature value
 */ 
int BMP280::updateTemperatureData(){
    char xlsb, lsb, msb; 
    int msbErr, lsbErr, xlsbErr; 
    xlsbErr = readData(BMP280_TEMP_XLSB, &xlsb, 1);
    lsbErr = readData(BMP280_TEMP_LSB, &lsb, 1);
    msbErr = readData(BMP280_TEMP_MSB, &msb, 1);
    
    int totalErr = xlsbErr + lsbErr + msbErr;

    //Shifts each byte into useful position
    int32_t rawTemperature = ((int32_t)msb << 12) | ((int32_t)lsb << 4 ) | ((int32_t)xlsb >> 4);
    values.temp_c = convert_temp(rawTemperature) - 3.5; // offsets temperature by experimentally gathered amount
    values.temp_f = values.temp_c * 9.0/5.0 + 32.0; 
    return totalErr; 
}

/**
 * @brief Retrieves and organizes pressure data - stores the values in the BMP_280VALUES struct
 * @return calulated pressure value
 */ 
int BMP280::updatePressureData(){
    char xlsb, lsb, msb;
    int msbErr, lsbErr, xlsbErr; 
    xlsbErr = readData(BMP280_PRESS_XLSB, &xlsb, 1); // Extra least significant byte 
    lsbErr = readData(BMP280_PRESS_LSB, &lsb, 1);// Least significant byte 
    msbErr = readData(BMP280_PRESS_MSB, &msb, 1); // Most significant byte 
    int totalErr = xlsbErr + lsbErr + msbErr;

    // Shifts each byte into useful position 
    uint32_t rawPressure = ((uint32_t)msb << 12) | ((uint32_t)lsb << 4 ) | ((uint32_t)xlsb >> 4);
    values.press_pa = convert_press(rawPressure) - 944.6; // offset from experimental data
    values.press_psi = values.press_pa * 0.000145038;
    return totalErr; 
}

/** 
 * @brief converts the raw sampling values to usable data using BMP280 calibration values
 * @param adc_t raw binary representing the temperature value 
 * @return a double of the temperature in celcius
 */
double BMP280::convert_temp(int32_t adc_T){
    int err = BMP280_CalibrateTemp();

    double var1, var2, T;
    var1 = (((double)adc_T)/16384.0 - ((double)c.dig_T1)/1024.0) * ((double)c.dig_T2);
    var2 = ((((double)adc_T)/131072.0 - ((double)c.dig_T1)/8192.0) * 
    (((double)adc_T)/131072.0 - ((double)c.dig_T1)/8192.0)) * ((double)c.dig_T3);
    t_fine = (int32_t)(var1 + var2);
    T = (var1 + var2)/5120.0;
    return T;
}

/** 
 * @brief calibrates the pressure based upon formulas from datasheet
 * @param adc_p raw binary representing the pressure value 
 * @return a double of the pressure in pascals
 */
double BMP280::convert_press(int32_t adc_P){
    int err = BMP280_CalibratePress(); 

    double var1, var2, p;
    var1 = ((double)t_fine/2.0) - 64000.0;
    var2 = var1 * var1 * ((double)c.dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)c.dig_P5) * 2.0;
    var2 = (var2/4.0)+(((double)c.dig_P4) * 65536.0);
    var1 = (((double)c.dig_P3) * var1 * var1 / 524288.0 + ((double)c.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0)*((double)c.dig_P1);
    if (var1 == 0.0){
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)c.dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)c.dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)c.dig_P7)) / 16.0;
    return p;
}

/** 
 * @brief updates sensor values (altitude, pressure, temperature)
 * @return number of errors accumlated from each update
 */
int BMP280::updateValues(){
    int errTemp = updateTemperatureData();
    int errPress  = updatePressureData();
    updateAltitudeM();
    return(errPress + errTemp);
}

/** 
 * @brief Calibrates the temperature oversampling using calibration registers 
          stores all calibration data in BMP_Calibration
 * @return 0 
 */
int BMP280::BMP280_CalibrateTemp(){
    char calib[6];
    readData(0x88, calib, 6);
    c.dig_T1 = calib[1] << 8 | calib[0];
    c.dig_T2 = (calib[3] << 8 | calib[2]);
    c.dig_T3 = (calib[5] << 8 | calib[4]);
    
    return 0;
}

/**
 * @brief Calibrates the pressure oversampling using calibration registers 
          stores all calibration data in BMP_Calibration 
 * @return 0 
 */
int BMP280::BMP280_CalibratePress(){
    char calib[18];

    // Read Calibration data 
    readData(0x8E, calib, 18);

    //Store Calibration data
    c.dig_P1 = calib[0] | calib[1] << 8;
    c.dig_P2 = calib[2] | calib[3] << 8;
    c.dig_P3 = calib[4] | calib[5] << 8;
    c.dig_P4 = calib[6] | calib[7] << 8;
    c.dig_P5 = calib[8] | calib[9] << 8;
    c.dig_P6 = calib[10] | calib[11] << 8;
    c.dig_P7 = calib[12] | calib[13] << 8;
    c.dig_P8 = calib[14] | calib[15] << 8;
    c.dig_P9 = calib[16] | calib[17] << 8;

    return 0;
}


/** 
 * @brief generates altitude data using the a kalman filter of the hypersometric and advanced hypersometric formula
 */
void BMP280::updateAltitudeM(){
    double univesalGasConst = 8.31432;
    double staticPress = 101325;
    double sealvlTemp_K = 15 + 273.15; 
    double tempLapseRate = -.0065;
    double gravity = 9.80665;
    double molarMassAir = .0289655;
    double h2 = (sealvlTemp_K/tempLapseRate) \
                        *(pow((values.press_pa/staticPress), \
                            -((univesalGasConst*tempLapseRate)/(gravity*molarMassAir)))-1.0); 

    double pressRatioTerm = pow((101325/values.press_pa),(1/5.257)) - 1.0;
    double temp_k = values.temp_c +273.15;
    double h1 = (pressRatioTerm*temp_k)/.0065; 
    values.altitude_m = .5*h1 +.5*h2;
}

/** 
 * @brief returns the BMP280 Values struct 
 * @return the BMP280_Values struct 
 */
BMP280_Values BMP280::getState() const{
     return values;
}