#include "mbed.h"
#include "Spoolcam.h"
using namespace std;

Spoolcam::Spoolcam(PinName output_pin) : trigger(output_pin) {
    this->recording = false;
}

/** @brief Raises recording line, starting recording process */ 
void Spoolcam::start_recording(void) {
    this->recording = true; 
    trigger.write(1);

}

/** @brief Lowers recording line, stopping recording process */
void Spoolcam::stop_recording(void){
    this->recording = false;
    trigger.write(0);
}

/** @return the recording status */
bool Spoolcam::is_recording(void) {
    return this->recording;
}

