#include "mbed.h"
#include "Spoolcam.h"
using namespace std;

Spoolcam::Spoolcam(PinName output_pin) : trigger(output_pin) {
    this->recording = false;
}

/** @brief Raises recording line, starting recording process */ 
void start_recording(void) {
    this->recording = true; 
    trigger.write(1);

}

/** @brief Lowers recording line, stopping recording process */
void stop_recording(void){
    this->recording = false;
    tigger.write(0);
}

/** @return the recording status */
bool is_recording(void) {
    return this->recording;
}

