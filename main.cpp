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

// Mutex_I2C i2c(PB_7, PB_8);
// ctrldRogallo ARES(&i2c);
// EUSBSerial pc;


// // // = # of pages, 4.55 hours at 1Hz
// // // 16 pages per sector

// // #define FLIGHT_PACKET_SIZE  sizeof(FlightPacket) // Size in bytes of one flight packet 
// // #define MAX_NUM_PACKETS     16384

// // /* SET NUMBER OF PACKETS TO LOG || for for 1.5 hours of logging, log 5400 packets */
// // #define NUM_PACKETS_TO_LOG  16000

// // #define DT_CTRL             0.01 // time step for PID controller to calculate derivative & integral

#define MCPS_ADDR 0x02 << 1


// // /*
// //  * SET NUMBER OF PACKETS TO LOG
// //  * for for 1.5 hours of logging, log 5400 packets
// //  */
// // // uint32_t numPackets = 16000;

// int main(void){

//     ARES.startAllSensorThreads(&pc); // start threads 

//     ThisThread::sleep_for(2s);

//     pc.printf("\nStarting testing sequence! Wait about 1 min\n");

//     FlightPacket state;
//     int deflection = 0x69; 
//     int counter = 0; 
//     int successful_sends = 0;
//     int successful_reads = 0;
//     int max = 5000;

//     pc.printf("Running %i tests at 50Hz\n", max);

//     while(counter < max){
//         ThisThread::sleep_for(20ms);
//         ARES.updateFlightPacket();
//         state = ARES.getState();
//         int ack = ARES.sendCtrl(deflection);
//         if (ack == 0) successful_sends++;
//         char* message = ARES.requestMotorPacket(); 
//         if(strcmp(message, "Bash") == 0) successful_reads++;
//         counter++;
//     }

//     pc.printf("Succesful Reads %i, Succesful Writes %i\n", successful_reads, successful_sends);
// }


Thread i2cThread; // setup thread

char i2c_tx_buf[32];
char i2c_rx_buf[32];


int i2c_handler(void) {

    I2CSlave slave(PB_7, PB_6);
    slave.address(MCPS_ADDR); // Expects shifted into correct position

    while(true) {
        int event = slave.receive();
        
        switch(event) {

            case I2CSlave::WriteAddressed: {
                int err = slave.read(i2c_rx_buf, sizeof(int));
                int num;
                memcpy(&num, i2c_rx_buf, sizeof(int));
                break;
            }

            case I2CSlave::ReadAddressed: {
                const char* message = "Bash";
                strcpy(i2c_tx_buf, message);
                slave.write(i2c_tx_buf, strlen(message) + 1);
                break;
            }

            default:
                // No event; just continue
                break;
        }
    }
}

int main(void){

    DigitalOut led(PC_13);
    
    ThisThread::sleep_for(1ms);

    led.write(1);

    i2cThread.start(i2c_handler);

    int counter = 0;
    while(counter < 5000) {
        counter++; 
        ThisThread::sleep_for(20ms);
    }

    if(counter == 5000) led.write(0);
}

