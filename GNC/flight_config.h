

// ====================================
//          FLIGHT PARAMETERS
// ====================================

const char FLIGHT_ID[8]                   = "ARES-02";
const int APOGEE_ALT_THRESHOLD_BUFFER     = 500;         // m
const float APOGEE_DETECTION_VELOCITY     = -1.0;       // m/s
const int APOGEE_COUNTER_THRESHOLD        = 100;         // counts
const int GROUNDED_ALT_THRESHOLD_BUFFER   = 30;         // m
const float GROUNDED_VELOCITY_RANGE       = 0.3;        // m/s
const float GROUNDED_COUNTER_THRESHOLD    = 2000;       // counts
const int INNER_SPIRAL_RADIUS             = 15;         // m
const int OUTER_SPIRAL_RADIUS             = 30;         // m


// ====================================
//       PASCO LANDING COORDINATES
// ====================================

// At Pasco TCR launch site, 100ft from the road on the west end of the crop circle
const double PASCO_TARGET_A_LAT           = 46.411306;  // deg
const double PASCO_TARGET_A_LON           = -119.018300;// deg
// southeast end of crop circle
const double PASCO_TARGET_B_LAT           = 46.410557;  // deg
const double PASCO_TARGET_B_LON           = -119.011137;// deg
// east end of crop circle
const double PASCO_TARGET_C_LAT           = 46.411373;  // deg
const double PASCO_TARGET_C_LON           = -119.010485;// deg

// ====================================
//       IMA LANDING COORDINATES
// ====================================

// 3min parking spot next to IMA parking lot
const double IMA_TARGET_LAT               = 47.659078;  // deg
const double IMA_TARGET_LON               = -122.298950;// deg

// ====================================
//       CHOSEN LANDING COORDINATES
// ====================================

const double ARES_TARGET_LAT              = PASCO_TARGET_C_LAT;
const double ARES_TARGET_LON              = PASCO_TARGET_C_LON;


// ====================================
//           PID CONTROLLER
// ====================================

// PID (tuned to receive an error in radians)
const float Kp                            = 1.0;
const float Ki                            = 0.02;
const float Kd                            = 0.0;

// ====================================
//             BIAS FILTER
// ====================================

const float ALPHA_ALT_PERCENT             = 0.05;       // frac

// ====================================
//              LOGGING
// ==================================== 

const int packet_save_incr                = 20;
const int flight_packet_size              = 256; 

// ====================================
//         GENERAL CONSTANTS
// ==================================== 

const int DEG_LLA_TO_M_CONVERSION         = 111111;
const float PI                            = 3.1415926535;
const float DEG_TO_RAD                    = PI/180.0;
const double MAX_DOUBLE                   = 0x1.fffffffffffffp+1023;

// ====================================
//       I2C PERIPHERAL ADDRESSES
// ==================================== 

const int MCPS_I2C_ADDR                   = 0x02 << 1; 
const int BMP_I2C_ADDR                    = 0xEE;
const int BNO_I2C_ADDR                    = 0x51;

// ====================================
//           GPS PIN DEFINITIONS
// ==================================== 

const PinName GPS_RX_PIN                  = PA_2;
const PinName GPS_TX_PIN                  = PA_3;

// ====================================
//           LED PIN DEFINITIONS
// ==================================== 

const PinName BLUE_LED_PIN                = PA_8;
const PinName GREEN_LED_PIN               = PA_15;

// ====================================
//        SPOOLCAM PIN DEFINITIONS
// ==================================== 

// Motor NRST 
const PinName SPOOLCAM_TRIGGER_PIN        = PB_1;
