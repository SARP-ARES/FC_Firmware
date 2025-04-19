#include "mbed.h"
#include "EUSBSerial.h"
#include "GPS.h"


const char* getTypeStr(NMEA_Type type) { // for printing to port
    switch(type) {
        case NMEA_GGA: {
            const char* type_str = "GGA";
            return type_str;
        }
        case NMEA_GSA: {
            const char* type_str = "GSA";
            return type_str;
        }
        case NMEA_GSV: {
            const char* type_str = "GSV";
            return type_str;
        }
        case NMEA_RMC: {
            const char* type_str = "RMC";
            return type_str;
        }
        case NMEA_VTG: {
            const char* type_str = "VTG";
            return type_str;
        }
    }
    const char* type_str = "NONE";
    return type_str;
}

int main(){
    DigitalOut led_G(PA_15);
    DigitalOut led_B(PA_8);

    EUSBSerial pc(true); // new class Extended USB Serial
    // BufferedSerial gps_serial(PA_2, PA_3);

    led_B.write(1);
    
    // setup timer for use in secondary actions
    Timer t;
    t.start();

    GPS gps(PA_2, PA_3); // instantiate

    char buf[256] = {0};
    int index = 0;
    int success;

    ThisThread::sleep_for(1000ms); // Wait for serial port to connect
    pc.printf("\n\nStarting GPS Testing...");

    ThisThread::sleep_for(1000ms); // Wait for serial port to connect
    // trying to turn on antenna status messages
    const char* buffy1 = "$CDCMD,33,1*7C\r\n";
    // const char* buffy2 = "$PGCMD,33,1*6C"; 
    int rslt1 = gps.serial.write(buffy1, 17);
    // int rslt2 = gps.serial.write(buffy2, 15);
    pc.printf("\nsent command to enable antenna status messages... result: %d", rslt1);


    while(true){
        // // BELOW SENDS FAKE MESSAGES TO TEST UPDATES


        // if (t.read_ms() >= 5000){
        //     t.reset(); // reset timer

        //     led_B = !led_B; // toggle LED

        //     // artifical GGA message
        //     // const char* msg = "$GPGGA,091626.000,2236.2791,N,12017.2818,E,1,10,1.00,8.8,M,18.7,M,,*66"; // GGA
        //     // const char* msg = "$GPGGA,091626.000,2236.2791,N,12017.2818,E,1,10,1.00,8.8,M,"; // testing smaller message
        //     // const char* msg = "$GNRMC,172740.000,A,4739.3344,N,12218.3884,W,0.11,154.71"; // RMC (cut down a little)
        //     const char* msg = "$GLGSA,A,3,86,77,,,,,,,,,,,2.16,1.93,0.96*11";
        //     // const char* msg = "$GPGSA,A,3,14,30,02,07,,,,,,,,,2.16,1.93,0.96*00"
        //     NMEA_Type msgType = gps.getMsgType(msg);
        //     int result = gps.update(msgType, msg);
        //     pc.printf("\n==================================");
        //     pc.printf("\n%s", msg); // write the message to serial port
        //     pc.printf("\nMessage Type: %d", msgType); // write the message type
        //     gpsState state = gps.getState();

        //     switch(msgType) {
        //         case NMEA_NA: {
        //             pc.printf("\ntype not found");
        //             break;
        //         }
        //         case NMEA_GGA: { // GGA
        //             pc.printf("\nUTC\t\t: %.3f \nFix\t\t: %d \nLattitude\t: %.4f %c \nLongitude\t: %.4f %c\nAltitude\t: %f\n", \
        //                     state.utc, state.fix, state.lat, state.latNS, state.lon, state.lonEW, state.alt);
        //             break;
        //         }
        //         case NMEA_GSA: { // GSA
        //             pc.printf("\nmode1\t\t: %c \nmode2\t\t: %d \nPDOP\t\t: %f \nHDOP\t\t: %f \nVDOP\t\t: %f", \
        //                     state.mode1, state.mode2, state.pdop, state.hdop, state.vdop);
        //             break;
        //         }
        //         case NMEA_RMC: { // RMC
        //             pc.printf("\nUTC\t\t: %.3f \nStatus\t\t: %c \nLattitude\t: %.4f %c \nLongitude\t: %f.4 %c\nGround Speed\t: %f knots \nHeading\t\t: %.3f degrees", \
        //                     state.utc, state.rmcStatus, state.lat, state.latNS, state.lon, state.lonEW, state.gspeed, state.heading);
        //             break;
        //         }
        //         case NMEA_VTG: { // VTG
        //             pc.printf("\nwoopy doopy VTG");
        //             break;
        //         }
        //     }
        //     pc.printf("\nitems matched\t: %d", result);
        // }


        //====================================================================================================
        // BELOW GETS AND PRINTS REAL GPS DATA USING LEGACY UPDATE METHOD
        

        // // read each character of the message and send to the corresponding buffer index
        // if (gps.serial.readable()) {
        //     gps.serial.read(&buf[index], 1);
        //     index ++;
        // }else{
        //     // pc.printf("\nGPS not readable...");
        // }

        // if (index != 0 && buf[index-1] == '\n') {
        //     buf[index] = 0; // terminate the string to negate leftovers
        //     NMEA_Type msgType = gps.getMsgType(buf);
        //     int result = gps.update(msgType, buf);; // TODO: combine getMsgType() & udpate() w/ wrapper
        //     pc.printf("\n===========================");
        //     pc.printf("\n%s", buf); // write the message to serial port
        //     const char* msgTypeStr = getTypeStr(msgType);
        //     pc.printf("Message Type\t: %s", msgTypeStr); // write the message type
        //     gpsState state = gps.getState();
        //     pc.printf("\nUTC\t\t: %f \nFix\t\t: %d \nmode2\t\t: %d \nLattitude\t: %f %c \nLongitude\t: %f %c\nAltitude\t: %f\nHeading\t\t: %.3f degrees \nPDOP\t\t: %f \nHDOP\t\t: %f \nVDOP\t\t: %f", \
        //                     state.utc, state.fix, state.mode2, state.lat, state.latNS, state.lon, state.lonEW, state.alt, state.heading, state.pdop, state.hdop, state.vdop);
        //     pc.printf("\nitems matched\t: %d", result);
        //     index = 0; // reset 

        //     // int rslt2 = gps.serial.write(buffy2, 15);
        //     // pc.printf("\nsent command to enable antenna status messages... result: %d", rslt2);
        // }


        //====================================================================================================
        // BELOW GETS AND PRINTS REAL GPS DATA USING bigUpdate();


        success = gps.bigUpdate();
        gpsState state = gps.getState();
        pc.printf("\n-----------------------------------------------");
        pc.printf("\nNumber of messages parsed with information: %d", success);
        pc.printf("\n-----------------------------------------------");
        pc.printf("\nUTC\t\t: %.3f \nFix\t\t: %d \nHeading\t\t: %.4f \nLattitude\t: %.4f %c \nLongitude\t: %.4f %c\nAltitude\t: %f \nmode1\t\t: %c \nmode2\t\t: %d \nPDOP\t\t: %f \nHDOP\t\t: %f \nVDOP\t\t: %f", \
                state.utc, state.fix, state.heading, state.lat, state.latNS, state.lon, state.lonEW, state.alt, state.mode1, state.mode2, state.pdop, state.hdop, state.vdop);
        pc.printf("\n===============================================\n\n");


        /* NOTES:
        - make above stuff in a function
        - getPacket()
        - put in loop / parse 6 messages all at once before updating state
        */
    }
}

