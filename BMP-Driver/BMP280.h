/* 
* Contains methods of the BMP280 Class and related enums and structs 
*/
#ifndef BMP280_H
#define BMP280_H

#include <cstdint> 
#include "mbed.h"
#include "USBSerial.h"
using namespace std;

struct BMP280_Values {
    double press_pa; // Pressure in Pascals 
    double temp_c; // Temperature in Celcius
    double altitude_m; // Altitude from Sea Level in m
    double press_psi; // Pressure in Psi
    double temp_f; // temp in farenheit
    double altitude_ft; // alt in ft 
    double altitude_h1; // alt from hypsometric 1 (m)
    double altitude_h2; // alt from hypsometric 2 (m)
    double altitude_b1; // alt from barometic (m)
};

struct BMP280_Calibration {
            // Temperature 
            uint16_t dig_T1;
            int16_t dig_T2, dig_T3;

            //Pressure 
            uint16_t dig_P1;
            int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
};


/*
*   Data Acquisition 
*/ 

class BMP280 { 
    public:
        /* Constructor 
        * @param Pin address of the SDA Pin on the processor 
        * @param Pin address of the SCL Pin on the processor 
        * @param The I2C address for the BMP280
        */
        BMP280(PinName SDA, PinName SCL, char addr); 

        /* Destructor 
        * Deletes I2C for this object
        */ 
        ~BMP280();


        // I2C functional methods
        int writeData(char regaddr, char data);
        int readData(char regaddr, char* data, uint8_t len);

        int updateValues(); // updates the current pressure and temperature values 

        int start(); // Awakens the BMP from slumber 
        int sleep(); // Sleeps the BMP 

    public: 

        int updatePressureData(); // updates the pressure value 
        int updateTemperatureData(); // updates the temperature value 
        void updateAltitudeH1();
        void updateAltitudeH2();
        // double updateAltitudeB1();
         // updates the current altitude based on temp and pressure

        BMP280_Values values; // Stores the nessecary values to be returned 
        BMP280_Calibration c; // Stores the calibration data nessecary for the sensor 

        bool owned;
        char addr; // Address of BMP280 
        I2C* i2c; // Initialize i2c object
        int32_t t_fine; // Nesscary temperature for calibration

        float readTemperatureData(); // Reads the temperature data from the register 
        float readPressureData();   // Reads the pressure data from the register
        double convert_temp(int32_t adc_T);  // Converts the raw temperature data to useable data
        double convert_press(int32_t adc_T); // converts the raw pressure data to usable data 
        int BMP280_CalibrateTemp(); // Calibrates the temperature values from the raw data collection 
        int BMP280_CalibratePress(); // Calibrates the pressure values from the raw data collection

};

#endif // BMP280_H