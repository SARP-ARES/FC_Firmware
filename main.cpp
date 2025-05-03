// Testing data logging with flash chip
#include "mbed.h"
#include "EUSBSerial.h"
#include "flash.h"
#include <cstdint>
#include "BNO055.h"


BNO055 bno(PB_7, PB_8, 0x51); 
EUSBSerial pc(true);
DigitalOut led_B(PA_8);


int main() {
    led_B.write(1);
    bno.setup();

   while (true) {
        bno055_vector_t euler = bno.getEuler();
        bno055_vector_t gyr = bno.getGyroscope();

       pc.printf("\nOrientation Yaw (deg)\t: %f"
                "\nOrientation Pitch (deg)\t: %f"
                "\nOrientation Roll (deg)\t: %f"
                "\nYaw speed (rad/s)\t: %f"
                "\nPitch speed (rad/s)\t: %f"
                "\nRoll speed (rad/s)\t: %f", euler.z, euler.y, euler.x, gyr.z, gyr.y, gyr.x);
        ThisThread::sleep_for(500ms);
   }
}

