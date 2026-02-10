/* 
* BMP280 Constants 
* Contains all variable names for all addresses 
*/ 

#ifndef BMP280_const_H
#define BMP280_const_H

const int BMP280_TEMP_XLSB = 0xFC;
const int BMP280_TEMP_LSB = 0xFB;
const int BMP280_TEMP_MSB = 0xFA;
const int BMP280_PRESS_XLSB = 0xF9;
const int BMP280_PRESS_MSB = 0xF7;
const int BMP280_PRESS_LSB = 0xF8;
const int BMP280_config = 0xF5;
const int BMP280_CTRL_MEAS = 0xF4;
const int BMP280_STATUS = 0xF3;
const int BMP280_RESET = 0xE0;
const int BMP280_ID = 0xd0;

#endif // BMP280_const_H