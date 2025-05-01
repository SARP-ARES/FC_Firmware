#include "mbed.h"
#include "EUSBSerial.h"
#include "I2CSerial.h"


int main() {
    
    EUSBSerial pc(true);
    I2CSerial slave(PB_7, PB_8, 0x32, true);
    DigitalOut led_R(PC_14);
    led_R.write(0);

    // // DEFINE CONTROL SEQUENCE
    // int32_t ctrl[100, -100, 0, 25, 50, 100];

    while (true){
        // // MASTER CODE
        // // WRITING CONTROL SEQUENCE
        // ThisThread::sleep_for(1s); // other processes
        // for (int i=0; i<5; i++) {
        //     master.write(ctrl[i], 4);
        //     ThisThread::sleep_for(5s); // change to 600s (10min) for flight
        // }

        // SLAVE CODE
        // ThisThread::sleep_for(1s); // other processes
        
        // read FC message into buffer
        char buf[256] = {0};
        for (int i = 0; i<5; i++) { // limit max packet reads per cycle
            slave.readline(buf, 256);
            //    .read(char* buf, size_t num_bytes); // reads num_bytes bytes into the buffer
            // handle messages here
        }

        // write the buffer back to FC
        slave.write(buf, 256); // for fix sized writes / reads or raw byte data  
    
    }
}
