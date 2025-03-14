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

    while (true) {
        if(t.read_ms() >= 4000){
            BMP280_Values values;
            
            values.temp_f = bmp280.getTemperature(); 
            values.press_psi = bmp280.getPressure(); 
            pc.printf("-------------------------------\n");
            pc.printf("Errors\t\t: %d\n", bmp280.updateValues());
            pc.printf("Temperature\t: %lf\nPressure\t: %lf\n", values.temp_f, values.press_psi);
            t.reset(); 
        }
    }
}