#include "mbed.h"
#include "EUSBSerial.h"
#include "ctrldRogallo.h"
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "flash.h"
#include "Mutex_I2C.h"
#include "flight_packet.h"
#include <iostream>
#include <cstring>

#define BMP_I2C 0xEE

Mutex_I2C i2c(PB_7, PB_8);
BMP280 bmp(&i2c, BMP_I2C);
EUSBSerial pc;
DigitalOut led_B(PA_8);
DigitalOut led_G(PA_15);
Thread thread;


// = # of pages, 4.55 hours at 1Hz
// 16 pages per sector

#define FLIGHT_PACKET_SIZE  sizeof(FlightPacket) // Size in bytes of one flight packet 
#define MAX_NUM_PACKETS     16384

/* SET NUMBER OF PACKETS TO LOG || for for 1.5 hours of logging, log 5400 packets */
#define NUM_PACKETS_TO_LOG  16000

#define DT_CTRL             0.01 // time step for PID controller to calculate derivative & integral

// Ts works
int main(void){
    bmp.start();
    BMPData val;
    while(true){
        ThisThread::sleep_for(500ms);
        bmp.update();
        val = bmp.getData(); 
        pc.printf("Pressure %lf\n", val.press_psi);
    }
}