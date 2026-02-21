#ifndef ROGALLO_H
#define ROGALLO_H
#include "GPS.h"
#include "BMP280.h"
#include "BNO055.h"
#include "PID.h"
#include "EUSBSerial.h"
#include "flash.h"
#include "flight_packet.h"
#include "Mutex_I2C.h"
#include <cstdint>

// Event flags uses bit masks, not indexes
const int BMP_FLAG        = (1UL << 0); // 1
const int BNO_FLAG        = (1UL << 1); // 2
const int GPS_FLAG        = (1UL << 2); // 4
const int LOGGING_FLAG    = (1UL << 3); // 8

// Finite State Machine Modes
typedef enum {
    FSM_IDLE,       // 0
    FSM_SEEKING,    // 1
    FSM_SPIRAL,     // 2
    FSM_GROUNDED,   // 3
} ModeFSM;

typedef struct {
    float leftPosition; 
    float rightPosition;
    float leftPull;
    float rightPull;
} motorPacket;

class ctrldRogallo {

    public:
        ctrldRogallo(Mutex_I2C* i2c);

        // -- Setters --
        void setLastFCcmd(float cmd);
        void setFSMMode(ModeFSM mode);

        // ---- thread handlers ----
        void startThreadGPS(); 
        void startThreadIMU();
        void startThreadBMP();
        void startAllSensorThreads(EUSBSerial* pc); // REMOVE ARG AFTER DEBUG COMPLETE
        void killThreadGPS();
        void killThreadIMU();
        void killThreadBMP();
        void killAllSensorThreads();
        void logDataLoop();
        void startLogging(flash* flash_mem, EUSBSerial* pc);
        void stopLogging();
        void stopAllThreads();

        // ---- state handlers ---
        const FlightPacket getState();
        const ModeFSM getMode();
        void updateFlightPacket();
        void resetFlightPacket();
        const uint16_t getPacketsLogged();

        // ---- other stuff ----
        float getElapsedSeconds();
        void setThreshold(); 
        void setTarget(double latitude, double longitude);
        void setPIDGains(float Kp, float Ki, float Kd);

        float getHeadingError();
        float getTargetHeading();
        float computeCtrl(float heading_error, float dt); // output in [-1, 1]

        // ---- MC/PS comms ----
        bool sendCtrl(float ctrl);
        bool requestMotorPacket();

        // ---- FSM mode ----
        uint32_t apogeeDetection(double prevAlt, double currAlt);
        uint32_t groundedDetection(double prevAlt, double currAlt);

        void printCompactState(EUSBSerial* pc);

    private:

        /* Drivers */
        BMP280 bmp;
        BNO055 bno;
        GPS gps;
        PID pid;
        Mutex_I2C* i2c; 
        flash* flash_mem = nullptr;
        
        uint32_t flash_addr;
        Timer flight_timer;
        FlightPacket state;
        ModeFSM mode;

        char rx_buf[32];

        /* Comms */ 
        motorPacket motor;

        /* Mutex */ 
        Mutex state_mutex;

        /* Threads */ 
        Thread thread_logging;
        Thread thread_imu;
        Thread thread_bmp;
        Thread thread_gps;

        /* Threading Flag Handler */
        EventFlags event_flags; 

        /* State local vars */ 
        bool apogeeDetected;
        uint32_t apogeeCounter;
        float alphaAlt;
        uint32_t groundedCounter; 
        float target_lat;
        float target_lon;
        float haversineCoordNorth;
        float haversineCoordEast;
        float distanceToTarget;
        uint32_t apogeeThreshold;
        uint32_t groundedThreshold; 
        uint32_t currentFlashAddress;
        uint16_t packets_logged;
        float prev_time;

        // ---- threads ----
        void bmpUpdateLoop();
        void imuUpdateLoop();
        void gpsUpdateLoop();

        // ---- navigation utilities ----
        float computeGreatCircleDistance(double lat_deg, double lon_deg, double lat_target_deg, double lon_target_deg);
        void updateDistanceToTarget(void);
        void updateGreatCircleDistance(void);
        bool isWithinTarget(void);
        float getFuzedAlt(float alt1, float alt2);
        void setAlphaAlt(float newAlphaAlt);
        void updateApogeeDetection();

        // nan helper 
        bool is_nan_safe(float f);
};

inline float ctrldRogallo::getElapsedSeconds() {
    using namespace std::chrono;
    return duration_cast<duration<float>>(flight_timer.elapsed_time()).count();
}

#endif