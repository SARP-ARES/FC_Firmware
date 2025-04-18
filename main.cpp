#include "mbed.h"
#include "BMP280.h"
#include "EUSBSerial.h"
#include "BMP280_const.h"

EUSBSerial pc;
int ack;   
int address;
I2C i2c(PB_7,PB_8);

void scanI2C() {
  for(address=1;address<127;address++) {    
    ack = i2c.write(address, "11", 1);
    if (ack == 0) {
       pc.printf("\tFound at %3d -- %3x\r\n", address,address);
    }    
    ThisThread::sleep_for(50ms);
  } 
}

// main() runs in its own thread in the OS
int main(){

    DigitalOut led_G(PA_15);
    DigitalOut led_B(PA_8);
    led_B.write(1);

    //EUSBSerial pc(true); //instantiate ESUBSerial 
    BMP280 bmp280(PB_7,PB_8,0x51); //instantiate 

    Timer t; 
    t.start();

    // Attempt to awaken the sensor
    ThisThread::sleep_for(1s);
    int result = bmp280.start();
    ThisThread::sleep_for(1s); // wait for serial port to connect
    
    if (result == 0) {
        pc.printf("\n=================================");
        pc.printf("\n=================================");
        pc.printf("\n\n\tThe BMP280 HAS RISEN\n");
        if(bmp280.wakeUpTemperature() == 0){
            pc.printf("\n\n\tTemp online\n");
        } else {
            pc.printf("\n\n\tNo temp :(\n");
        }

        pc.printf("\n=================================\n");

        if(bmp280.wakeUpPressure() == 0){
            pc.printf("\n\n\tTHE Press IS OPERATIONAL\n");
        } else {
            pc.printf("\n\n\t press no feel so good\n"); 
        }
        pc.printf("\n=================================");
        pc.printf("\n=================================\n");
        
    }else {
        pc.printf("\n========================================");
        pc.printf("\n========================================");
        pc.printf("\n\n\tThe BMP280 remains asleep\n");
        pc.printf("\n========================================");
        pc.printf("\n========================================\n");   
    }

    while (true) {
        if(t.read_ms() >= 4000){
            int err = bmp280.updateValues();
            float temp_f = bmp280.getTemperature(); 
            float press_psi = bmp280.getPressure(); 
            pc.printf("\n-------------------------------");
            pc.printf("\nErrors\t\t: %d\nTemperature\t: %lf\nPressure\t: %lf\n", err, temp_f, press_psi);
            t.reset(); 
        }
    }
}