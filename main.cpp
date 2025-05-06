// Testing MCPS-FC Comms
#include "mbed.h"
#include "EUSBSerial.h"
#include "I2CSerial.h"

#include "Motor.h"
#include "PID.h"
#include "Distributor.h"
#include "MotorData.h"


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

void motor_control() {



    MotorData packet = {
        0.0,
        0.0,
        0.0,
        0.0,
    };

    DigitalIn startPin(PB_7);
    ThisThread::sleep_for(5000ms);
    while (startPin.read() == 1) {
        ThisThread::sleep_for(100ms);
    }



    DigitalOut led1(PC_14);
    led1.write(1);

    PID pid(0.001, 5,  0.1);
    I2CSerial mcpsToFc(PB_7, PB_8, 0x32, true);

    // Motor motor1(PB_5, PB_3, PA_11, PA_10, PA_12, PA_9, &pid); // left motor WITH PA_14 SUBSTITUTION, A AND B ENCODER CHANNELS SWITCHED BECAUSE ONLY A WORKS
    Motor motor2(PA_5, PA_6, PB_14, PB_15, PB_13, PA_8, &pid); // right motor

    ThisThread::sleep_for(30s);
    motor2.motorPower(1);
    ThisThread::sleep_for(16s);
    motor2.motorPower(0);

    // int seqLength = 3;
    // float arr1[3] = {720, 0, 0};
    // float arr2[3] = {0, 720, 0};
    // int seconds = 5;
    // int dt = 10;
    // int repspersec = 100;

    // float power1 = 0;
    // float power2 = 0;

    
    // for (int i = 0; i < seqLength; i++) {

    //     for (int j = 0; j < seconds*repspersec; j++) {
    //         power1 = -pid.compute(motor1.getDegrees(), arr1[i], dt);
    //         power2 = -pid.compute(motor2.getDegrees(), arr2[i], dt);
    //         // slave.printf("Power: %d %\n", (int) (power*100));
    //         motor1.motorPower(power1);
    //         motor2.motorPower(power2);
    //         if (j % 50 == 0) {
    //         }
    //         ThisThread::sleep_for(10ms);
    //     }
    // }


    // for (int i = 0; i < 500; i++) {
    //     power = -pid.compute(motor1.getDegrees(), -360, 10);
    //     motor1.motorPower(-0.5);
    //     ThisThread::sleep_for(10ms);
    // }

    led1.write(0);

    
    // for (int i = 0; true; i++) {
    //     ThisThread::sleep_for(10ms);
    //     power = -pid.compute(motor2.getDegrees(), 7000, 10);
    //     motor2.motorPower(power);
    //     if (i % 50 == 0) {
    //         // Write entire data packet (struct)
    //         packet.delta1_deg = 7000 - motor2.getDegrees();
    //         packet.delta2_deg = 0;
    //         packet.pwm_motor1 = power;
    //         packet.pwm_motor2 = 0;
    //         mcpsToFc.write(reinterpret_cast<char*>(&packet), sizeof(MotorData));
    //     }
        
        // slave.printf("degrees: %d%\n", motor1.getDegrees()*100);
    //}
}

int main() {
    motor_control();
}