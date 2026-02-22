#include "mbed.h"
#include "EUSBSerial.h"
#include "GPS.h"
#include "GPS_CMD.h"
// #include <Adafruit_GPS>

EUSBSerial pc(true); // new class Extended USB Serial

GPS gps(PA_2, PA_3, pc); // instantiate

#define GPS_BUF_SIZE 256
char gps_buf[GPS_BUF_SIZE] = {0};
int gps_index = 0;

DigitalOut led_G(PA_15);
DigitalOut led_B(PA_8);

// BufferedSerial gps_serial(PA_2, PA_3);

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

// Process a completed NMEA line
void process_gps_line(GPS* gps, const char* line) {
    NMEA_Type msgType = gps->getMsgType(line);
    int result = gps->update(msgType, line);

    pc.printf("\n===========================\n");
    pc.printf("%s", line);
    pc.printf("Message Type\t: %s\n", getTypeStr(msgType));
}

// Poll GPS and fill buffer until newline, then process
void poll_gps(GPS* gps) {
    static char buf[GPS_BUF_SIZE];
    static int index = 0;

    while (gps->serial.readable()) {
        char c;
        if (gps->serial.read(&c, 1) == 1) {
            // store character
            if (index < GPS_BUF_SIZE - 1) {
                buf[index++] = c;
            }

            // End of line reached
            if (c == '\n') {
                buf[index] = '\0';  // terminate C string
                process_gps_line(gps, buf);
                index = 0;          // reset buffer
            }
        }
    }
}


int main(){
    led_B.write(1);
    
    gps.setOriginECEFr(47.664612, -122.303568, 60); // set lat/lon/alt origin
    // setup timer for use in secondary actions
    Timer t;
 
    /*
    - center of red square (between benches): 47.655821, -122.309691
    - triple brick stack red sq: 47.656229, -122.309831
    - Pasco launch rail: 
    - 47.664612, -122.303568
    */
 
    // Thread thred; // thread to automatically set origin if needed
    // thred.start()

    
    int index = 0;
    int success;

    ThisThread::sleep_for(2s); // Wait for serial port to connect
    pc.printf("\n\nStarting GPS Testing...");

    // trying to turn on antenna status messages
    // const char* buffy1 = "$CDCMD,33,1*7C";
    // const char* buffy2 = "$PGCMD,33,1*6C"; 
    // const char* tenHurtzCmd = "$PMTK220,100*2F";
    // const char* antenna_status_cmd = "$PMTK300,200,0,0,0,0*2F";


    // int rslt1 = gps.serial.write(antenna_status_cmd, sizeof(tenHurtzCmd));
    // pc.printf("\nsent command to enable antenna status messages... result: %d", rslt1);

/* 
    // These commands should just work

    int rslt1 = gps.serial.write(CMD_BAUD_9600.cmd, CMD_BAUD_9600.len);
    ThisThread::sleep_for(100ms);
    gps.serial.set_baud(9600); // also change baud rate of USART2 on STM32 so the GPS and STM can communicate
    ThisThread::sleep_for(100ms);
    pc.printf("\n\n...Wrote 9600 baud rate cmd... result: %d", rslt1);
    // 9600, 38400, 57600, 115200
    int rslt2 = gps.serial.write(CMD_UPDATE_1HZ.cmd, CMD_UPDATE_1HZ.len);
    ThisThread::sleep_for(100ms);
    pc.printf("\n\n...Wrote 1Hz cmd... result: %d", rslt2);

    int rslt3 = gps.serial.write(CMD_FIXCTL_1HZ.cmd, CMD_FIXCTL_1HZ.len);
    ThisThread::sleep_for(100ms);
    pc.printf("\n\n...Wrote 1Hz fix cmd... result: %d", rslt3);
*/

// normal cmd setting start
    // int rslt1 = gps.serial.write(CMD_BAUD_115200.cmd, CMD_BAUD_115200.len);
    // ThisThread::sleep_for(100ms);
    // gps.serial.set_baud(115200); // also change baud rate of USART2 on STM32 so the GPS and STM can communicate
    // ThisThread::sleep_for(100ms);
    // pc.printf("\n\n...Wrote 115200 baud rate cmd... result: %d", rslt1);
    // // 9600, 38400, 57600, 115200
    // int rslt2 = gps.serial.write(CMD_UPDATE_10HZ.cmd, CMD_UPDATE_10HZ.len);
    // ThisThread::sleep_for(100ms);
    // pc.printf("\n\n...Wrote 10Hz cmd... result: %d", rslt2);

    // int rslt3 = gps.serial.write(CMD_FIXCTL_10HZ.cmd, CMD_FIXCTL_10HZ.len);
    // ThisThread::sleep_for(100ms);
    // pc.printf("\n\n...Wrote 10Hz fix cmd... result: %d", rslt3);

    // //Test NEMA sentence filter (so GPS only sends RMC and GGA)
    // int rslt4 = gps.serial.write(CMD_NMEA_NECESSARY.cmd, CMD_NMEA_NECESSARY.len);
    // ThisThread::sleep_for(100ms);
    // pc.printf("\n\n...Wrote NMEA sentence set cmd... result: %d", rslt4);
// normal cmd setting end

// // function cmd setting start
gps.set_logging_rate(10);

// // function cmd setting end

    int rslt5 = gps.serial.write(CMD_ENABLE_ANTENNA_STATUS.cmd, CMD_ENABLE_ANTENNA_STATUS.len);
    ThisThread::sleep_for(100ms);
    pc.printf("\n\n...Wrote Enable Antenna status cmd sentence set cmd... result: %d", rslt5);


    ThisThread::sleep_for(5s);
    led_B.write(0);
    ThisThread::sleep_for(500ms);
    led_B.write(1);
    t.start(); //t is a timer

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


        // // ====================================================================================================
        // // BELOW GETS AND PRINTS REAL GPS DATA USING SIMPLE UPDATE METHOD
        

        
        // //read each character of the message and send to the corresponding buffer index
        // if (gps.serial.readable()) {
        //     gps.serial.read(&gps_buf[index], 1);
        //     index ++;
        // }else{
        //     // pc.printf("\nGPS not readable...");
        // }
        
        // if (index != 0 && gps_buf[index-1] == '\n') {
        //     gps_buf[index] = 0; // terminate the string to negate leftovers
        //     NMEA_Type msgType = gps.getMsgType(gps_buf);
        //     int result = gps.update(msgType, gps_buf);; // TODO: combine getMsgType() & udpate() w/ wrapper
        //     pc.printf("\n===========================");
        //     pc.printf("\n%s", gps_buf); // write the message to serial port
        //     const char* msgTypeStr = getTypeStr(msgType);
        //     pc.printf("Message Type\t: %s", msgTypeStr); // write the message type
        //     gpsState state = gps.getState();
        //     // pc.printf("\nUTC\t\t: %f \nFix\t\t: %d \nmode2\t\t: %d \nLattitude\t: %f %c \nLongitude\t: %f %c\nAltitude\t: %f\nHeading\t\t: %.3f degrees \nPDOP\t\t: %f \nHDOP\t\t: %f \nVDOP\t\t: %f \nAntenna Status\t: %d", \
        //     //                 state.utc, state.fix, state.mode2, state.lat, state.latNS, state.lon, state.lonEW, state.alt, state.heading, state.pdop, state.hdop, state.vdop, state.antenna_status);
        //     // pc.printf("\nitems matched\t: %d", result);
        //     index = 0; // reset    

        //     // if (t.read_ms() > 5000) { // get antenna status every 5s
        //     //     gps.serial.write(cmd_antenna_status, 25);
        //     // } 
        // }
        
             

        // if (t.read_ms() > 5000) { // get antenna status every 5s
        //     gps.serial.write(cmd_antenna_status, 25);
        // } 
    


        // THIS MAKES 10HZ WORK 
        // // Try process line & polling functions
        // poll_gps(&gps);   // keep draining GPS data
        

        //====================================================================================================
        // BELOW GETS AND PRINTS REAL GPS DATA USING bigUpdate();


        // pc.printf("\n\nentering main loop...\n");
        success = gps.bigUpdate();

        // pc.printf("did bigUpdate...\n");

        gpsState state = gps.getState();
        // pc.printf("retrieved GPS state...\n");

        // posLTP pos = gps.getPosLTP();
        // pc.printf("retrieved LTP position...\n");

        // posECEFr origin = gps.getOriginECEFr();
        // pc.printf("retrieved ECEFr origin...\n");
        
        float lat_deg = state.lat;
        float lon_deg = state.lon;
        float lat_rad = state.lat * pi / 180;
        float lon_rad = state.lon * pi / 180;

        // pc.printf("\n-----------------------------------------------");
        // pc.printf("\nLat (deg)\t: %.4f \nLon (deg)\t: %.4f \nLat (rad)\t: %.4f \nLon (rad)\t: %.4f", lat_deg, lon_deg, lat_rad, lon_rad);
        // pc.printf("\n-----------------------------------------------");

        pc.printf("\n===============================================");
        pc.printf("\nNumber of messages parsed with information: %d", success);
        pc.printf("\n-----------------------------------------------");
        pc.printf("\nUTC\t\t: %.3f \nFix\t\t: %d \nHeading\t\t: %.3f \nLat, Lon\t: %f, %f deg \nAltitude\t: %.3f m \nmode1\t\t: %c \nmode2\t\t: %d \nPDOP\t\t: %f \nHDOP\t\t: %f \nVDOP\t\t: %f \nAntenna Status\t: %d", \
                state.utc, state.fix, state.heading, state.lat, state.lon, state.alt, state.mode1, state.mode2, state.pdop, state.hdop, state.vdop, state.antenna_status);
        pc.printf("\n===============================================\n\n");


        /* NOTES:
        - make above stuff in a function
        - getPacket()
        - put in loop / parse 6 messages all at once before updating state
        */
    }
}