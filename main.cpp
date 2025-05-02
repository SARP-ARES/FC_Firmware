// Testing MCPS-FC Comms
#include "mbed.h"
#include "EUSBSerial.h"
#include "I2CSerial.h"



void flightComputer() {
    I2CSerial master(PB_3, PB_10, 0x32, false);
    EUSBSerial pc(false);
    ThisThread::sleep_for(1s);
    DigitalOut led_B(PA_8);
    led_B.write(1);
    int ctrl[6] = {100, -100, 0, 25, 50, 100};


    while (true){
        // WRITING CONTROL SEQUENCE
        ThisThread::sleep_for(1s); // other processes

        char buf[256] = {0};
        for (int i = 0; i<6; i++){
            // write control to MCPS
            pc.printf("\nSending: %+04d\n", ctrl[i]);
            master.printf("%+04d\n", ctrl[i]);
            ThisThread::sleep_for(200ms);

                        
            // MCPS should send it back
            if (master.readline(buf, 256)) {
                pc.printf("Recieving: %s\n", buf);
            } else {
                pc.printf("Nothing Recieved...\n");
            }
        
            ThisThread::sleep_for(2s); // change to 600s (10min) for flight
        }
    }
        
}

void mcps() {
    I2CSerial slave(PB_7, PB_8, 0x32, true);
    DigitalOut led_R(PC_14);
    led_R.write(0);

    while (true) {
        // read FC message into buffer
        char buf[256] = {0};
        for (int i = 0; i<5; i++) { // limit max packet reads per cycle
            
            if (!slave.readline(buf, 256)) {
                break;
            }
            //    .read(char* buf, size_t num_bytes); // reads num_bytes bytes into the buffer
            // handle messages here
            slave.printf("MCPS->FC: %s\n", buf);
            // slave.write(buf, 256);

        }

        // write the buffer back to FC
        // for fix sized writes / reads or raw byte data  
    }
}


void mcp_log() {
    Timer t;
    size_t count = 0;

    DigitalOut led(PA_8);
    led.write(1);

    I2CSerial master(PB_3, PB_10, 0x32, false);
    EUSBSerial pc(0x3232, 0x1);

    ThisThread::sleep_for(1s);

    t.start();
    while (true) {
        char buf[256] = {0};
        if (master.readline(buf, 256)) {
            pc.printf("(MCP) %s", buf);
            t.reset();
        }

        if (t.read_ms() > 5000) {
            pc.printf("(FC) KeepAlive %d\n", count);
            count ++;
            t.reset();
        }
    }
}

void mcp() {
    I2CSerial slave(PB_7, PB_8, 0x32, true);

    DigitalOut led_R(PC_14);
    led_R.write(1);

    int c = 0;
    while (true) {
        ThisThread::sleep_for(1s);

        slave.printf("Hello World: %d\n", c);
        c++;
    }
}

int main() {
    mcp();
}
