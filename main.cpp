// Testing data logging with flash chip
#include "ThisThread.h"
#include "mbed.h"
#include "EUSBSerial.h"
#include "flash.h"
#include <cstdint>


int main() {
    
    // instantiate flash chip
    flash fc(PA_7, PA_6, PA_5, PA_4); // MOSI, MISO, SCLK, CS (Flight Computer)

    EUSBSerial pc(true);
    DigitalOut led_B(PA_8);
    led_B.write(1);

    pc.printf("CHECK");
    // make test data
    float lat = 90.0;
    float lon = 180.0;
    int pSize_float_bytes = 8; // 2 floats = 8 bytes

    int packetSize = 8;

    // uint32_t flashAddress = 0; // start writing at address 0
    pc.printf("Starting flash chip testing...");
    while (true){
        // TESTING...
        // THIS IS IN THE LOOP ON PURPOSE TO ONLY WRITE TO THE FIRST [packetSize] BYTES
        uint32_t flashAddress = 0; // start writing at address 0

        // // initialize packet buffer
        // uint8_t dataLog[packetSize];

        // // initialize arrays to store floats as bytes
        // uint8_t lat_bytes[4];
        // uint8_t lon_bytes[4];

        // // turn all float data into bytes
        // float2Byte(lat_bytes, lat);
        // float2Byte(lon_bytes, lon);

        // // store all bytes in dataLog
        // int index = 0;
        // for (int i=0; i<4; i++){
        //     dataLog[index] = lat_bytes[i];
        //     index++;
        // }
        // for (int i=0; i<4; i++){
        //     dataLog[index] = lon_bytes[i];
        //     index++;
        // }

        // // write datalog and bump flashAddress to the next empty address
        // fc.write(flashAddress, dataLog, packetSize);
        // flashAddress = flashAddress + packetSize;


        // OR I CAN JUST USE WRITENUM
        pc.printf("\n==================================");
        flashAddress = fc.writeNum(flashAddress, lat); // bytes 0-3
        flashAddress = fc.writeNum(flashAddress, lon); // bytes 4-7
        

        uint8_t buf[packetSize];
        fc.read(0, buf, packetSize);
        pc.printf("\nData: %s", (const char*)buf);
        ThisThread::sleep_for(2s);
        // fc.write(flashAddress, packet_floats, pSize_floats)

        // flashAddress = flashAddress + pSize_floats // step to the address after the packet

        // fc.write(packet_ints)
        // flashAddress = flashAddress + pSize_ints

        // fc.write(packet_chars)
        // flashAddress = flashAddress + pSize_chars

    }


}

