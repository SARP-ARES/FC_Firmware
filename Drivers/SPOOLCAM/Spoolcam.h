#ifndef SPOOLCAM_H
#define SPOOLCAM_H

#include "mbed.h"

class Spoolcam {

    public:

        Spoolcam(PinName output_pin);

        void start_recording(void);
        void stop_recording(void);
        bool is_recording(void);

    private:

        bool recording;
        DigitalOut trigger; 

};

#endif