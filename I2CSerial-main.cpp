/* I2CSerial test functionaility
 * adjust main() to change between master and slave
 */

#include "mbed.h"
#include "EUSBSerial.h"


EUSBSerial pc(0x3232, 0x1);
I2C i2c(PB_7, PB_8); 

int ack; 
int address;  
void scanI2C() {
  for(address=0;address<255;address++) {    
    ack = i2c.write(address, "11", 1);
    if (ack == 0) {
       pc.printf("\tFound at %3d -- %3x\r\n", address,address);
    }    
  } 
}

int main() {
    while (true) {
        pc.printf("Hello World!\n");
        scanI2C();
        ThisThread::sleep_for(500ms);
    }
}