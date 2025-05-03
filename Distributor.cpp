#include "mbed.h"
#include <string>
#include "Distributor.h"
#include <cmath>
#include "I2CSerial.h"



Distributor::Distributor() {}

std::pair<float, float> Distributor::getMotorOutputs()  {

    char buf[DISTRIBUTOR_BUFFER_SIZE] = {' ', '0', '0', '0'};
    
    int digit1 = 100*( buf[1] - '0' );
    int digit2 = 10*( buf[2] - '0' );
    int digit3 = buf[3] - '0';
    int sum = digit1 + digit2 + digit3;
    if (buf[0] == '-') {sum*=-1;} 

    printf("Distributor turn angle: %d\n", sum);


    float ctrl = ( (float) sum ) / 100.0;
    // printf("ctrl: %d\n", (int) (ctrl*100));
    
    float leftPull = 0.5 * (-ctrl) + 0.5;
    float rightPull = 0.5 * ctrl + 0.5;

    return std::make_pair(leftPull, rightPull);
}