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

Mutex_I2C i2c(PB_7, PB_8);
ctrldRogallo ARES(&i2c);
EUSBSerial pc;


// = # of pages, 4.55 hours at 1Hz
// 16 pages per sector

#define FLIGHT_PACKET_SIZE  sizeof(FlightPacket) // Size in bytes of one flight packet 
#define MAX_NUM_PACKETS     16384

/* SET NUMBER OF PACKETS TO LOG || for for 1.5 hours of logging, log 5400 packets */
#define NUM_PACKETS_TO_LOG  16000

#define DT_CTRL             0.01 // time step for PID controller to calculate derivative & integral

#define MCPS_ADDR 0x02 << 1


/*
 * SET NUMBER OF PACKETS TO LOG
 * for for 1.5 hours of logging, log 5400 packets
 */
// uint32_t numPackets = 16000;

int main(void){

    ARES.startAllSensorThreads(&pc);

    FlightPacket state;
    int deflection = 0x69; 
    while(true){
        ThisThread::sleep_for(500ms);
        ARES.updateFlightPacket(); 
        ARES.printCompactState(&pc);
        int ack = ARES.sendCtrl(deflection);
        if(ack == 0){
            pc.printf("Success\n");
        }
        char* message = ARES.requestMotorPacket(); 
        pc.printf("la message %s\n", message);
    }
}



// MCPS fake main 
// int main() {

//     I2CSlave slave(PB_7, PB_6);
//     slave.address(MCPS_ADDR); // Expects shifted into correct position

//     char tx_buf[32];
//     char rx_buf[32];

//     while(true) {
//         int event = slave.receive();
        
//         switch(event) {

//             case I2CSlave::WriteAddressed: {
//                 int err = slave.read(rx_buf, sizeof(int));
//                 int num;
//                 memcpy(&num, rx_buf, sizeof(int));
//                 break;
//             }

//             case I2CSlave::ReadAddressed: {
//                 const char* message = "Bash";
//                 strcpy(tx_buf, message);
//                 slave.write(tx_buf, strlen(message) + 1);
//                 break;
//             }

//             default:
//                 // No event; just continue
//                 break;
//         }
//     }
// }

