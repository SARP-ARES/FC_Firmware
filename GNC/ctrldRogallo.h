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

// Finite State Machine Modes
typedef enum {
    FSM_IDLE,       // 0
    FSM_SEEKING,    // 1
    FSM_SPIRAL,     // 2
    FSM_GROUNDED,   // 3
} ModeFSM;


class ctrldRogallo {

    private:
        BMP280 bmp;
        BNO055 bno;
        GPS gps;
        PID pid;
        flash* flash_mem = nullptr;
        uint32_t* flash_addr = nullptr;
        Timer flight_timer;
        FlightPacket state;
        ModeFSM mode;
        Mutex state_mutex;
        Mutex imu_mutex;
        Mutex bmp_mutex;
        Mutex gps_mutex;

        Thread thread_logging;
        Thread thread_imu;
        Thread thread_bmp;
        Thread thread_gps;

        uint32_t apogeeDetected;
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

        // sensor thread handling
        void bmpUpdateLoop();
        void imuUpdateLoop();
        void gpsUpdateLoop();

        float computeHaversine(double lat_deg, double lon_deg, double lat_target_deg, double lon_target_deg);
        void updateDistanceToTarget(void);
        void updateHaversineCoords(void);
        bool isWithinTarget(void);
        void setModeFSM(ModeFSM mode);
        float getFuzedAlt(float alt1, float alt2);
        void setAlphaAlt(float newAlphaAlt);
        void updateApogeeDetection();

    public:
        ctrldRogallo(Mutex_I2C* i2c);
        
        // thread handling
        void startThreadGPS(EUSBSerial* pc); // REMOVE ARG AFTER DEBUG COMPLETE
        void startThreadIMU();
        void startThreadBMP();
        void startAllSensorThreads(EUSBSerial* pc); // REMOVE ARG AFTER DEBUG COMPLETE
        void logDataLoop();
        void startLogging(flash* flash_mem, uint32_t* flash_addr);

        // state handlers
        const FlightPacket getState();
        void updateFlightPacket();
        void resetFlightPacket();

        float getElapsedSeconds();
        void setThreshold(); 
        void setTarget(double latitude, double longitude);
        void setPIDGains(float Kp, float Ki, float Kd);

        float getHeadingError();
        float getTargetHeading();
        float computeCtrl(float heading_error, float dt); // output in [-1, 1]

        // MC/PS comms
        uint8_t sendCtrl(float ctrl);
        void requestMotorPacket(void);

        uint32_t apogeeDetection(double prevAlt, double currAlt);
        uint32_t groundedDetection(double prevAlt, double currAlt);
        void printCompactState(EUSBSerial* pc);
};

inline float ctrldRogallo::getElapsedSeconds() {
    using namespace std::chrono;
    return duration_cast<duration<float>>(flight_timer.elapsed_time()).count();
}

#endif