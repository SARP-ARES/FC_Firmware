#include "mbed.h"
#include "EUSBSerial.h"
#include "GPS.h"


int main(){
    DigitalOut led_G(PA_15);
    DigitalOut led_B(PA_8);

    EUSBSerial pc(true); // new class Extended USB Serial
    BufferedSerial gps_serial(PA_2, PA_3);

    led_B.write(0);
    
    // setup timer for use in secondary actions
    Timer t;
    t.start();

    GPS gps; // instantiate

    char buf[256] = {0};
    int index = 0;

    ThisThread::sleep_for(1000ms); // Wait for serial port to connect
    pc.printf("\n\nStarting GPS Testing...");
    while(true){
        // pc.printf("..::: entered while-loop :::..");
        // TESTING; SENDING FAKE MESSAGES
        if (t.read_ms() >= 5000){
            t.reset(); // reset timer

            led_B = !led_B; // toggle LED

            // artifical GGA message
            // const char* msg = "$GPGGA,091626.000,2236.2791,N,12017.2818,E,1,10,1.00,8.8,M,18.7,M,,*66"; // GGA
            // const char* msg = "$GPGGA,091626.000,2236.2791,N,12017.2818,E,1,10,1.00,8.8,M,"; // testing smaller message
            const char* msg = "$GNRMC,172740.000,A,4739.3344,N,12218.3884,W,0.11,154.71"; // RMC (cut down a little)
            NMEA_Type msgType = gps.getMsgType(msg);
            int result = gps.update(msgType, msg);
            pc.printf("\n==================================");
            pc.printf("\n%s", msg); // write the message to serial port
            pc.printf("\nMessage Type: %d", msgType); // write the message type
            gpsState state = gps.getState();

            switch(msgType) {
                case NMEA_NA: {
                    pc.printf("\ntype not found");
                    break;
                }
                case NMEA_GGA: { // GGA
                    pc.printf("\nUTC\t\t: %.3f \nFix\t\t: %d \nLattitude\t: %.4f %c \nLongitude\t: %.4f %c\nAltitude\t: %f\n", \
                            state.utc, state.fix, state.lat, state.latNS, state.lon, state.lonEW, state.alt);
                    break;
                }
                case NMEA_GSA: { // GSA
                    pc.printf("\nwoopy doopy GSA");
                    break;
                }
                case NMEA_RMC: { // RMC
                    pc.printf("\nUTC\t\t: %.3f \nStatus\t\t: %c \nLattitude\t: %.4f %c \nLongitude\t: %f.4 %c\nGround Speed\t: %f knots \nHeading\t\t: %.3f degrees", \
                            state.utc, state.rmcStatus, state.lat, state.latNS, state.lon, state.lonEW, state.gspeed, state.heading);
                    break;
                }
                case NMEA_VTG: { // VTG
                    pc.printf("\nwoopy doopy VTG");
                    break;
                }
            }
            pc.printf("\nitems matched\t: %d", result);
        }

        // // read each character of the message and send to the corresponding buffer index
        // if (gps_serial.readable()) {
        //     gps_serial.read(&buf[index], 1);
        //     index ++;
        // }

        // if (index != 0 && buf[index-1] == '\n') {
        //     buf[index] = 0; // terminate the string to negate leftovers
        //     NMEA_Type msgType = gps.getMsgType(buf);
        //     gpsDebug debug = gps.update(msgType, buf); // TODO: combine getMsgType() & udpate() w/ wrapper
        //     pc.printf("==================================\n");
        //     pc.printf("%s", buf); // write the message to serial port
        //     pc.printf("Message Type: %d\n", msgType); // write the message type
        //     gpsState state = gps.getState();
        //     pc.printf("UTC\t\t: %f \nFix\t\t: %d \nLattitude\t: %f %c \nLongitude\t: %f %c\nAltitude\t: %f\n", \
        //                     state.utc, state.fix, state.lat, state.latNS, state.lon, state.lonEW, state.alt);
        //     pc.printf("DEBUG\t\t: %d\n", debug);
        //     index = 0; // reset 
        // }
    }
}

