// Testing data logging with flash chip
#include "mbed.h"
#include "EUSBSerial.h"
#include "I2CSerial.h"



void flightComputer() {
    EUSBSerial pc();
    I2CSerial master(PB_3, PB_10, 0x32, false);
    int ctrl[6] = {100, -100, 0, 25, 50, 100};

    while (true){
        
        // WRITING CONTROL SEQUENCE
        ThisThread::sleep_for(1s); // other processes

        char buf[256] = {0};
        for (int i = 0; i<6; i++){
            // write control to MCPS
            pc.printf("Sending: %+03d", ctrl[i]);
            master.printf("%+03d", ctrl[i]);
            ThisThread::sleep_for(100ms);

            // MCPS should send it back
            master.read(buf, 256);
            pc.printf("Recieving: %s", buf);

            ThisThread::sleep_for(2s); // change to 600s (10min) for flight
        }
    }
        
}

void mcps() {
    I2CSerial slave(PB_7, PB_8, 0x32, true);
    DigitalOut led_R(PC_14);
    led_R.write(1);

    while (true) {
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


int main() {
    mcps();
}
