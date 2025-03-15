/* I2CSerial test functionaility
 * adjust main() to change between master and slave
 */

 #include "mbed.h"
 #include "EUSBSerial.h"
 #include "I2CSerial.h"
 
 
 void master() {
     EUSBSerial pc(0x3232, 0x1);
 
     ThisThread::sleep_for(1s);
 
     I2CSerial ser(PB_3, PB_10, 0x32);
 
     while (true) {
         
 
         char buf[100] = {0};
         bool res = ser.readline(buf, 99);
         if (res) {
             pc.printf("<MCP> %s", buf);
             ThisThread::sleep_for(20ms);
         }
     }
 
 }
 
 
 void _led_blink() {
     DigitalOut led(PC_13);
     while (true) {
         led = !led;
         ThisThread::sleep_for(1s);
     }
 }
 
 void slave() {
     Thread t;
     t.start(_led_blink);
 
 
 
     I2CSerial ser(PB_7, PB_8, 0x32, true);
 
     int i = 0;
 
     while (true) {
         ser.printf("test %d\n", i);
         i ++;
 
         ThisThread::sleep_for(1s);
     }
     
 }
 
 
 int main() {
     master();
 }