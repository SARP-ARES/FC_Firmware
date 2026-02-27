#include "PID.h"

PID::PID(float Kp, float Ki, float Kd) {
    positiveLast = true;
    errorLast = 0;
    integralError = 0;
    firstCompute = true;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}


float PID::compute(float error, float dt) {
    // (INTEGRAL ANTI-WINDUP METHOD 1) set integral to zero while output is saturated to avoid windup
    // This is done in the if-statement at the end of the method.

    // // (INTEGRAL ANTI-WINDUP METHOD 2) reset integralError if the error switches sign 
    // if ((positiveLast && error < 0) || (!positiveLast && error > 0)) {
    //     integralError = 0;
    //     positiveLast = !positiveLast;
    // }

    // accumulate integral error
    integralError += error * dt;

    float P = -Kp * error;
    float I = -Ki * integralError;
    float D = -Kd * (error - errorLast) / dt;

    errorLast = error;

    // ignore derivative on first compute 
    // errorLast would jump from zero and spike output
    if (firstCompute) {
        D = 0;
        firstCompute = false;
    }

    float output = P + I + D;

    // saturation
    if (output > 1) {
        integralError = 0; // anti-windup (control is saturated)
        return 1;
    } else if (output < -1) {
        integralError = 0; // anti-windup (control is saturated))
        return -1;
    }
    return output;
}


void PID::updateGains(float Kp, float Ki, float Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}