#include <cstdint> // For std::uint16_t

const int START_BYTE = 0xAA;
const int RESPONSE_BYTE = 0xBB;
const int ERROR_BYTE = 0xEE;

const int BNO055_I2C_ADDR_HI = 0x29;
const int BNO055_I2C_ADDR_LO = 0x28;
const int BNO055_I2C_ADDR    = BNO055_I2C_ADDR_LO;

const int BNO055_READ_TIMEOUT  = 100;
const int BNO055_WRITE_TIMEOUT = 10;

const int ERROR_WRITE_SUCCESS    = 0x01;  // Everything working as expected
const int ERROR_WRITE_FAIL       = 0x03;  // Check connection, protocol settings and operation more of BNO055
const int ERROR_REGMAP_INV_ADDR  = 0x04;   // Invalid register address
const int ERROR_REGMAP_WRITE_DIS = 0x05;  // Register is read-only
const int ERROR_WRONG_START_BYTE = 0x06;  // Check if the first byte
const int ERROR_BUS_OVERRUN_ERR  = 0x07;  // Resend the command, BNO055 was not able to clear the receive buffer
const int ERROR_MAX_LEN_ERR      = 0x08;  // Split the command, max fire size cw be up to 128 bytes
const int ERROR_MIN_LEN_ERR      = 0x09;  // Min lengthwf data is less than 1
const int ERROR_RECV_CHAR_TIMEOUT = 0x0A;  // Decrease the waiting time between wnding of two bytes of one frame

const int REG_WRITE = 0x00;
const int REG_READ = 0x01;

// Page 0
const int BNO055_ID = 0xA0;
const int BNO055_CHIP_ID = 0x00;        // value: wA0
const int BNO055_ACC_ID = 0x01;      // value: wFB
const int BNO055_MAG_ID = 0x02;      // value: = 0x32
const int BNO055_GYRO_ID = 0x03;        // value: = 0x0F
const int BNO055_SW_REV_ID_LSB = 0x04;  // value: = 0x08
const int BNO055_SW_REV_ID_MSB = 0x05;  // value: = 0x03
const int BNO055_BL_REV_ID = 0x06;      // N/A
const int BNO055_ACC_DATA_X_LSB = 0x08;
const int BNO055_ACC_DATA_X_MSB = 0x09;
const int BNO055_ACC_DATA_Y_LSB = 0x0A;
const int BNO055_ACC_DATA_Y_MSB = 0x0B;
const int BNO055_ACC_DATA_Z_LSB = 0x0C;
const int BNO055_ACC_DATA_Z_MSB = 0x0D;
const int BNO055_MAG_DATA_X_LSB = 0x0E;
const int BNO055_MAG_DATA_X_MSB = 0x0F;
const int BNO055_MAG_DATA_Y_LSB = 0x10;
const int BNO055_MAG_DATA_Y_MSB = 0x11;
const int BNO055_MAG_DATA_Z_LSB = 0x12;
const int BNO055_MAG_DATA_Z_MSB = 0x13;
const int BNO055_GYR_DATA_X_LSB = 0x14;
const int BNO055_GYR_DATA_X_MSB = 0x15;
const int BNO055_GYR_DATA_Y_LSB = 0x16;
const int BNO055_GYR_DATA_Y_MSB = 0x17;
const int BNO055_GYR_DATA_Z_LSB = 0x18;
const int BNO055_GYR_DATA_Z_MSB = 0x19;
const int BNO055_EUL_HEADING_LSB = 0x1A;
const int BNO055_EUL_HEADING_MSB = 0x1B;
const int BNO055_EUL_ROLL_LSB = 0x1C;
const int BNO055_EUL_ROLL_MSB = 0x1D;
const int BNO055_EUL_PITCH_LSB = 0x1E;
const int BNO055_EUL_PITCH_MSB = 0x1F;
const int BNO055_QUA_DATA_W_LSB = 0x20;
const int BNO055_QUA_DATA_W_MSB = 0x21;
const int BNO055_QUA_DATA_X_LSB = 0x22;
const int BNO055_QUA_DATA_X_MSB = 0x23;
const int BNO055_QUA_DATA_Y_LSB = 0x24;
const int BNO055_QUA_DATA_Y_MSB = 0x25;
const int BNO055_QUA_DATA_Z_LSB = 0x26;
const int BNO055_QUA_DATA_Z_MSB = 0x27;
const int BNO055_LIA_DATA_X_LSB = 0x28;
const int BNO055_LIA_DATA_X_MSB = 0x29;
const int BNO055_LIA_DATA_Y_LSB = 0x2A;
const int BNO055_LIA_DATA_Y_MSB = 0x2B;
const int BNO055_LIA_DATA_Z_LSB = 0x2C;
const int BNO055_LIA_DATA_Z_MSB = 0x2D;
const int BNO055_GRV_DATA_X_LSB = 0x2E;
const int BNO055_GRV_DATA_X_MSB = 0x2F;
const int BNO055_GRV_DATA_Y_LSB = 0x30;
const int BNO055_GRV_DATA_Y_MSB = 0x31;
const int BNO055_GRV_DATA_Z_LSB = 0x32;
const int BNO055_GRV_DATA_Z_MSB = 0x33;
const int BNO055_TEMP = 0x34;
const int BNO055_CALIB_STAT = 0x35;
const int BNO055_ST_RESULT = 0x36;
const int BNO055_INT_STATUS = 0x37;
const int BNO055_SYS_CLK_STATUS = 0x38;
const int BNO055_SYS_STATUS = 0x39;
const int BNO055_SYS_ERR = 0x3A;
const int BNO055_UNIT_SEL = 0x3B;
const int BNO055_OPR_MODE = 0x3D;
const int BNO055_PWR_MODE = 0x3E;
const int BNO055_SYS_TRIGGER = 0x3F;
const int BNO055_TEMP_SOURCE = 0x40;
const int BNO055_AXIS_MAP_CONFIG = 0x41;
const int BNO055_AXIS_MAP_SIGN = 0x42;
const int BNO055_ACC_OFFSET_X_LSB = 0x55;
const int BNO055_ACC_OFFSET_X_MSB = 0x56;
const int BNO055_ACC_OFFSET_Y_LSB = 0x57;
const int BNO055_ACC_OFFSET_Y_MSB = 0x58;
const int BNO055_ACC_OFFSET_Z_LSB = 0x59;
const int BNO055_ACC_OFFSET_Z_MSB = 0x5A;
const int BNO055_MAG_OFFSET_X_LSB = 0x5B;
const int BNO055_MAG_OFFSET_X_MSB = 0x5C;
const int BNO055_MAG_OFFSET_Y_LSB = 0x5D;
const int BNO055_MAG_OFFSET_Y_MSB = 0x5E;
const int BNO055_MAG_OFFSET_Z_LSB = 0x5F;
const int BNO055_MAG_OFFSET_Z_MSB = 0x60;
const int BNO055_GYR_OFFSET_X_LSB = 0x61;
const int BNO055_GYR_OFFSET_X_MSB = 0x62;
const int BNO055_GYR_OFFSET_Y_LSB = 0x63;
const int BNO055_GYR_OFFSET_Y_MSB = 0x64;
const int BNO055_GYR_OFFSET_Z_LSB = 0x65;
const int BNO055_GYR_OFFSET_Z_MSB = 0x66;
const int BNO055_ACC_RADIUS_LSB = 0x67;
const int BNO055_ACC_RADIUS_MSB = 0x68;
const int BNO055_MAG_RADIUS_LSB = 0x69;
const int BNO055_MAG_RADIUS_MSB = 0x6A;
//
// BNO055 Page 1
const int BNO055_PAGE_ID = 0x07;
const int BNO055_ACC_CONFIG = 0x08;
const int BNO055_MAG_CONFIG = 0x09;
const int BNO055_GYRO_CONFIG_0 = 0x0A;
const int BNO055_GYRO_CONFIG_1 = 0x0B;
const int BNO055_ACC_SLEEP_CONFIG = 0x0C;
const int BNO055_GYR_SLEEP_CONFIG = 0x0D;
const int BNO055_INT_MSK = 0x0F;
const int BNO055_INT_EN = 0x10;
const int BNO055_ACC_AM_THRES = 0x11;
const int BNO055_ACC_INT_SETTINGS = 0x12;
const int BNO055_ACC_HG_DURATION = 0x13;
const int BNO055_ACC_HG_THRESH = 0x14;
const int BNO055_ACC_NM_THRESH = 0x15;
const int BNO055_ACC_NM_SET = 0x16;
const int BNO055_GYR_INT_SETTINGS = 0x17;
const int BNO055_GYR_HR_X_SET = 0x18;
const int BNO055_GYR_DUR_X = 0x19;
const int BNO055_GYR_HR_Y_SET = 0x1A;
const int BNO055_GYR_DUR_Y = 0x1B;
const int BNO055_GYR_HR_Z_SET = 0x1C;
const int BNO055_GYR_DUR_Z = 0x1D;
const int BNO055_GYR_AM_THRESH = 0x1E;
const int BNO055_GYR_AM_SET = 0x1F;

const int BNO055_OPERATION_MODE_CONFIG       = 0x00;
// Sensor Mode
const int BNO055_OPERATION_MODE_ACCONLY     = 0x01;
const int BNO055_OPERATION_MODE_MAGONLY     = 0x02;
const int BNO055_OPERATION_MODE_GYRONLY     = 0x03;
const int BNO055_OPERATION_MODE_ACCMAG     = 0x04;
const int BNO055_OPERATION_MODE_ACCGYRO    = 0x05;
const int BNO055_OPERATION_MODE_MAGGYRO    = 0x06;
const int BNO055_OPERATION_MODE_AMG        = 0x07;
// Fusion Mode
const int BNO055_OPERATION_MODE_IMU        = 0x08;
const int BNO055_OPERATION_MODE_COMPASS    = 0x09;
const int BNO055_OPERATION_MODE_M4G        = 0x0A;
const int BNO055_OPERATION_MODE_NDOF_FMC_OFF  = 0x0B;
const int BNO055_OPERATION_MODE_NDOF       = 0x0C;


// Data read scale
static constexpr std::uint16_t accelScale       = 100;
static constexpr std::uint16_t tempScale        = 1;
static constexpr std::uint16_t angularRateScale = 16;
static constexpr std::uint16_t eulerScale       = 16;
static constexpr std::uint16_t magScale         = 16;
static constexpr std::uint16_t quaScale         = (1 << 14); // 2^14
