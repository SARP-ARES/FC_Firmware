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
#include <cstdio>
#include <cstring>
#include "rtos.h"


EUSBSerial pc;
Mutex_I2C i2c(PB_7, PB_8);
ctrldRogallo ARES(&i2c);
DigitalOut led(PA_8);

motorPacket motor;

// // // = # of pages, 4.55 hours at 1Hz
// // // 16 pages per sector

// // #define FLIGHT_PACKET_SIZE  sizeof(FlightPacket) // Size in bytes of one flight packet 
// // #define MAX_NUM_PACKETS     16384

// // /* SET NUMBER OF PACKETS TO LOG || for for 1.5 hours of logging, log 5400 packets */
// // #define NUM_PACKETS_TO_LOG  16000

// // #define DT_CTRL             0.01 // time step for PID controller to calculate derivative & integral

// // /*
// //  * SET NUMBER OF PACKETS TO LOG
// //  * for for 1.5 hours of logging, log 5400 packets
// //  */
// // // uint32_t numPackets = 16000;

bool verify_mp(motorPacket m){
    return m.leftDegrees  == 1 && \
           m.rightDegrees == 2 && \
           m.leftPower    == 3 && \
           m.rightPower   == 4; 
}

int main(void){

    ARES.startAllSensorThreads(&pc); // start threads 

    ThisThread::sleep_for(2s);

    pc.printf("\nStarting testing sequence! Wait about 1 min\n");

    FlightPacket state;
    float deflection = -1;
    int counter = 0; 
    int successful_sends = 0;
    int successful_reads = 0;
    int max = 2500;

    pc.printf("Running %i tests at 50Hz\n", max);

    bool going_up = true;

    while(counter < max){
        ThisThread::sleep_for(20ms);

        // test normal functions
        ARES.updateFlightPacket();
        state = ARES.getState();

        int ack = ARES.sendCtrl(deflection);

        if (deflection > 0)  led.write(1);
        else                 led.write(0);

        if ((going_up && deflection > 1) || (!going_up && deflection < -1)) going_up = !going_up;
        
        if (going_up)    deflection += 0.01;
        else             deflection -= 0.01;

        if (ack == 0) successful_sends++;
        bool success = ARES.requestMotorPacket(&motor); 
        if(verify_mp(motor)) successful_reads++;
        counter++;
    }

    pc.printf("Succesful Reads %i, Succesful Writes %i\n", successful_reads, successful_sends);
}

