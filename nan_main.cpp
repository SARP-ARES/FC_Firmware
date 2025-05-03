
// #include "mbed.h"
// #include "EUSBSerial.h"


// int main() {

//     EUSBSerial pc(false);

//     float f1 = 25; // SET THIS
//     float f2 = NAN; // SET THIS
//     float fuzedAlt = NAN; 
//     float alphaAlt = 0.95; 
//     // float nanogram = NAN;

//     while (true) {

//         if (f1 == f1 && f2 == f2){
//             fuzedAlt = f1*(1-alphaAlt) + f2*alphaAlt;
//         } else if (f1 == f1) {
//             fuzedAlt = f1;
//         }else if (f2 == f2){
//             fuzedAlt = f2; 
//         } else {
//             fuzedAlt = NAN; // both bmp and gps are giving nans
//         }
//         pc.printf("fuzedAlt: %f\n", fuzedAlt);

//         ThisThread::sleep_for(2s);
//     }
// }