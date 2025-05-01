// Testing data logging with flash chip
#include "mbed.h"
#include "EUSBSerial.h"
#include "I2CSerial.h"


int main() {
    
    EUSBSerial pc(true);
    I2CSerial master(PB_7, PB_8, 0x32, true);
    DigitalOut led_R(PC_14);
    led_R.write(0);

    // DEFINE CONTROL SEQUENCE
    const char ctrl[30] = "_100,-100,_000,_025,_050,_100";
    // each ctrl message is 4 chars followed by a comma = 5 chars

    while (true){
        // MASTER CODE
        // WRITING CONTROL SEQUENCE
        ThisThread::sleep_for(1s); // other processes

        char buf[256] = {0};
        for (int i = 0; i<30; i+5){
            // write control to MCPS
            pc.printf("Sending: %s", ctrl[i]);
            master.write(&ctrl[i], 4);
            ThisThread::sleep_for(100ms);

            // MCPS should send it back
            master.read(buf, 256);
            pc.printf("Recieving: %s", buf);

            ThisThread::sleep_for(5s); // change to 600s (10min) for flight
        }


        // // SLAVE CODE
        // // ThisThread::sleep_for(1s); // other processes
        
        // // read FC message into buffer
        // char buf[256] = {0};
        // for (int i = 0; i<5; i++) { // limit max packet reads per cycle
        //     slave.readline(buf, 256);
        //     //    .read(char* buf, size_t num_bytes); // reads num_bytes bytes into the buffer
        //     // handle messages here
        // }

        // // write the buffer back to FC
        // slave.write(buf, 256); // for fix sized writes / reads or raw byte data  
    
    }
}
