#include "mbed.h"
#include "BMP280.h"
#include "EUSBSerial.h"
#include "BMP280_const.h"

EUSBSerial pc(0x3232, 0x1);

//EUSBSerial pc;
int ack;   
int address;
I2C i2c(PB_7,PB_8);


void scanI2C() {
  for(address=1;address<255;address++) {    
    ack = i2c.write(address, "11", 1);
    if (ack == 0) {
       pc.printf("\tFound at %3d -- %3x\r\n", address,address);
    }    
  } 
}

void _kick_watchdog() {
    Watchdog &watchdog = Watchdog::get_instance();

    // ensure watchdog is started
    if (!watchdog.is_running()){
        watchdog.start(30000);
    }


    while (true) {
        watchdog.kick();
        ThisThread::sleep_for(25s);
    }
}

// main() runs in its own thread in the OS
int main() {
    Thread wd;
    wd.start(_kick_watchdog);

    DigitalOut led_G(PA_15);
    DigitalOut led_B(PA_8);
    led_B.write(1);

    //EUSBSerial pc(true); //instantiate ESUBSerial 
    BMP280 bmp280(PB_7,PB_8,0xEE); //instantiate 

    Timer t; 
    t.start();

    // Attempt to awaken the sensor
    ThisThread::sleep_for(1s);
    int result = bmp280.start();
    ThisThread::sleep_for(1s); // wait for serial port to connect
    

    while (true) {
        if(t.read_ms() >= 400) {

            char status = 0;
            bmp280.readData(0xD0, &status, 1);

            pc.printf("id: %d\n", status);

            int err = bmp280.updateValues();
            float temp_f = bmp280.getTemperature(); 
            float press_psi = bmp280.getPressure(); 
            float altitude_ft = bmp280.getAltitude();
            pc.printf("\n=========================");
            pc.printf("\nErrors\t\t: %d\nTemperature (F)\t: %lf\nPressure (psi)\t: %lf\n Altiude (ft)\t: %lf ", err, temp_f, press_psi, altitude_ft);
            t.reset(); 
        }
    }
}