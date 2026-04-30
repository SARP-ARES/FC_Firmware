#ifndef SPOOLCAM_H
#define SPOOLCAM_H

#include "mbed.h"

class Spoolcam {

    public:

        Spoolcam(PinName output_pin);

        void start_recording();
        void stop_recording();
        bool is_recording();

    private:

        bool recording;
        DigitalOut trigger; 

}

#endif