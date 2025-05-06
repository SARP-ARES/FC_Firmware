// Class which controls DC motor with ethan's mosftet motor board
#ifndef _MOTOR_H_
#define _MOTOR_H_


#include "mbed.h"
#include "PID.h"

class Motor {
    private:
        // Defining constants from arduino
        const int LOW = 0;
        const int HIGH = 1;

        DigitalOut motorPin1;
        DigitalOut motorPin3;
        PwmOut motorPin2;
        PwmOut motorPin4;
        InterruptIn encoderA;
        InterruptIn encoderB;
        DigitalOut led;

        // Encoder state boolean
        bool aUp = false;
        bool bUp = false;

        const float PI = 3.1415926535;

        const int encoderChannels = 1; // Number of channels being used
        volatile int encoderCounter = 0; // Counter for the encoder ticks
        const int countsPerRev = 32 * encoderChannels;
        const float gearRatio = 150; // Might need to be updated for different motors
        const float spoolDiameter = 0.5; // Spool Diameter IN INCHES. Sidenote in realidad this will not be const
        const float MAX_DEFLECTION = 36; // INCHES

        const float totalCounts = countsPerRev * gearRatio; // Total counts per rotation of motor output shaft
        long position;
        float angle;
        float rotations;
        bool powerPositive;

        PID* motorPID;


        void aRiseCallback();
        void bRiseCallback();
        void aFallCallback();
        void bFallCallback();


    public:

        void updateGlobals();

        int getEncoderA();
        int getEncoderB();

        Motor(PinName PIN_A, PinName PIN_B, PinName MOTOR_1, PinName MOTOR_2, PinName MOTOR_3,
                PinName MOTOR_4, PID* pidObject);

        void motorPower(float power);

        void spinDegrees(int degrees);

        int getDegrees();

        long getDisplacement();

        void lineTo(float retraction, int delay);
        
        int getEncoderTicks();
};

#endif //_MOTOR_H_