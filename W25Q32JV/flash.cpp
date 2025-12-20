#include "mbed.h"
#include "flash.h"

#ifndef FLASH_ENABLE_RESET
#define FLASH_ENABLE_RESET  0x66
#endif

#ifndef FLASH_RESET
#define FLASH_RESET         0x99
#endif

/**
 * Constructor: Initializes SPI interface and chip select pin.
 * @param mosi - SPI MOSI pin
 * @param miso - SPI MISO pin
 * @param sclk - SPI Clock pin
 * @param csPin - Chip Select pin
 */
flash::flash(PinName mosi, PinName miso, PinName sclk, PinName csPin)
    : _spi(mosi, miso, sclk), _cs(csPin, 1) {
    _spi.format(8, 0);           // 8-bit frame, mode 0
    _spi.frequency(1000000);     // 1 MHz SPI clock
}


/**
 * Constructor: Initializes SPI interface, chip select pin, and EUSBSerial object.
 * @param mosi - SPI MOSI pin
 * @param miso - SPI MISO pin
 * @param sclk - SPI Clock pin
 * @param csPin - Chip Select pin
 * @param pc - pointer to EUSBSerial object for printing data
 */
flash::flash(PinName mosi, PinName miso, PinName sclk, PinName csPin, EUSBSerial* pc)
    : _spi(mosi, miso, sclk), _cs(csPin, 1), pc(pc) {
    _spi.format(8, 0);           // 8-bit frame, mode 0
    _spi.frequency(1000000);     // 1 MHz SPI clock
}

/**
 * Drives chip select (CS) line low to initiate communication.
 */
void flash::csLow() {
    _cs = 0;
    wait_us(5); // Chip select stabilization delay
}

/**
 * Releases chip select (CS) line to end communication.
 */
void flash::csHigh() {
    wait_us(5); // Deselect stabilization delay
    _cs = 1;
}

/**
 * Writes a buffer of data to a specific address in flash memory.
 * @param address - 24-bit target address
 * @param buffer - Pointer to data buffer
 * @param length - Number of bytes to write
 */
uint32_t flash::write(uint32_t address, const uint8_t *buffer, size_t length) {
    enableWrite(); // Required before any write

    uint8_t cmd[4];
    cmd[0] = 0x02; // Page Program command
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    ScopedLock<Mutex> lock(this->flash_lock);
    csLow();
    _spi.write((const char *)cmd, 4, NULL, 0);
    _spi.write((const char *)buffer, length, NULL, 0);
    csHigh();

    wait_us(5000); // Allow time for write cycle
    return address + length;
}

/**
 * Reads data from a specific address in flash memory.
 * @param address - 24-bit source address
 * @param buffer - Buffer to store read data
 * @param length - Number of bytes to read
 */
void flash::read(uint32_t address, uint8_t *buffer, size_t length) {
    uint8_t cmd[4];
    cmd[0] = 0x03; // Read Data command
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    
    ScopedLock<Mutex> lock(this->flash_lock);
    csLow();
    _spi.write((const char *)cmd, 4, NULL, 0);
    _spi.write(NULL, 0, (char *)buffer, length); // Only receive data
    csHigh();
}

/**
 * Reads a single byte from flash memory.
 * @param address - Address to read from
 * @return Value of the byte
 */
uint8_t flash::readByte(uint32_t address) {
    uint8_t data = 0xFF;
    read(address, &data, 1);
    return data;
}

/**
 * Writes a single byte to flash memory.
 * @param address - Address to write to
 * @param data - Byte value to write
 */
void flash::writeByte(uint32_t address, uint8_t data) {
    write(address, &data, 1);
}


/**
 * Wait for the flash chip to finish writing
 * @returns integer indicating timeout (1) or proper funciton (0)
 */
int flash::waitForWriteToFinish() {
    Timer t;
    uint8_t status; 
    t.start();
    uint8_t read_status_cmd = 0x05;
    t.start();
    while(true){
        wait_us(1000);
        
        csLow();
        _spi.write((const char *)&read_status_cmd, 1, (char *)&status, 1);
        csHigh();

        // ensure "write-in-progress" flag (at bit zero) is zero (not writing)
        if((status & 0b1) == 0){ 
            return 0; // no error
        }

        // timeout
        if (t.elapsed_time() > 30s) {
            return 1; // error
        }
    }
}

/**
 * Erases a 4KB sector at the given address.
 * @param address - Address within the sector to erase
 * @returns integer indicating timeout (1) or proper funciton (0)
 */
int flash::eraseSector(uint32_t address) {
    enableWrite();

    uint8_t cmd[4];
    cmd[0] = 0x20; // Sector Erase command
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    csLow();
    _spi.write((const char *)&cmd, 4, NULL, 0);
    csHigh();

    // // wait for erase to finish and return status
    // return waitForWriteToFinish();
    wait_us(500000);
    return 0;
}

/**
 * Erases all sectors.
 */
int flash::eraseAll() {
    Timer t;
    enableWrite();
    uint8_t erase_all_cmd = 0xC7; 

    csLow();
    _spi.write((const char *)&erase_all_cmd, 1, NULL, 0);
    csHigh();

    return waitForWriteToFinish();
}


/**
 * Sends Write Enable command to allow write/erase operations.
 */
void flash::enableWrite() {
    uint8_t cmd = 0x06; // Write Enable
    csLow();
    _spi.write((const char *)&cmd, 1, NULL, 0);
    csHigh();
    wait_us(5000);
}

/**
 * Sends Write Disable command to block write operations.
 */
void flash::disableWrite() {
    uint8_t cmd = 0x04; // Write Disable
    csLow();
    _spi.write((const char *)&cmd, 1, NULL, 0);
    csHigh();
    wait_us(5000);
}


/**
 * Resets the flash chip using the two-command reset sequence.
 */
void flash::reset() {
    uint8_t cmd;

    cmd = FLASH_ENABLE_RESET;
    csLow();
    _spi.write((const char *)&cmd, 1, NULL, 0);
    csHigh();
    ThisThread::sleep_for(5ms);

    cmd = FLASH_RESET;
    csLow();
    _spi.write((const char *)&cmd, 1, NULL, 0);
    csHigh();

    ThisThread::sleep_for(100ms); // Wait for reset completion
}

/**
 * Writes a 32-bit float to flash memory at specified address.
 * @param address - Flash memory address
 * @param data - Float value to store
 * @return address + 4 (indexed to the next empty address)
 */
uint32_t flash::writeNum(uint32_t address, float data) {
    uint8_t tempBytes[4];
    float2Byte(tempBytes, data);
    write(address, tempBytes, 4);
    return address + 4; // index to the next empty flash address
}

/**
 * Reads a 32-bit float from flash memory.
 * @param address - Flash memory address
 * @return Float value read from memory
 */
float flash::readNum(uint32_t address) {
    uint8_t rData[4];
    read(address, rData, 4);
    return bytes2float(rData);
}


uint16_t flash::getNumPacketsWritten() {
    uint16_t count;
    // Read current count (stored in the last two bytes of flash memory)
    read(0x3FFFFE, reinterpret_cast<uint8_t*>(&count), 2);
    wait_us(100);
    return count;
}

// Write entire data packet (struct)
uint32_t flash::writePacket(uint32_t address, const FlightPacket& pkt) {
    // write the packet
    write(address, reinterpret_cast<const uint8_t*>(&pkt), sizeof(FlightPacket));

    // figure out the current number of packets
    uint16_t count = getNumPacketsWritten();
    if (count == 0xFFFF) {
        // If it's the default erased value, write a packet and set count to 1
        { // lock to make sure erased count isn't read elsewhere before new count is written
            ScopedLock<Mutex> lock(this->flash_lock);
            eraseSector(0x3FFFFE);
            count = 1;
            write(0x3FFFFE, reinterpret_cast<uint8_t*>(&count), 2);
        }
    } else {
        // If there are packets already, just increment the counter
        { // lock to make sure erased count isn't read elsewhere before new count is written
            ScopedLock<Mutex> lock(this->flash_lock);
            // erase sector before writing
            eraseSector(0x3FFFFE);
            count += 1;
            write(0x3FFFFE, reinterpret_cast<uint8_t*>(&count), 2);
        }
    }
    return address + 256; // increment write address to the next page
}

// Read packet
uint32_t flash::readPacket(uint32_t address, FlightPacket& pkt) {
    read(address, reinterpret_cast<uint8_t*>(&pkt), sizeof(FlightPacket));
    return address + 256; // next page
}

void flash::printCSVHeader() {
    pc->printf(
        "timestamp_timer,"
        "timestamp_gps,"
        "fsm_mode,"
        "gps_fix,"
        "gps_antenna_status,"
        "heading_deg,"
        "target_heading_deg,"
        "target_heading_deg,"
        "h_speed_m_s,"
        "v_speed_m_s,"
        "latitude_deg,"
        "longitude_deg,"
        "altitude_gps_m,"
        "altitude_bmp_m,"
        "altitude_m,"
        "pos_east_m,"
        "pos_north_m,"
        "distance_to_target_m,"
        "temp_c,"
        "pressure_pa,"
        "delta1_deg,"
        "delta1_m,"
        "delta2_deg,"
        "delta2_m,"
        "delta_a,"
        "delta_s,"
        "pwm_motor1,"
        "pwm_motor2,"
        "fc_cmd,"
        "apogee_counter,"
        "apogee_detected,"
        "yaw_rate,"
        "pitch_rate,"
        "roll_rate,"
        "bno_acc_x,"
        "bno_acc_y,"
        "bno_acc_z,"
        "bno_mag_x,"
        "bno_mag_y,"
        "bno_mag_z,"
        "bno_eul_x,"
        "bno_eul_y,"
        "bno_eul_z,"
        "bno_lin_x,"
        "bno_lin_y,"
        "bno_lin_z,"
        "bno_grav_x,"
        "bno_grav_y,"
        "bno_grav_z,"
        "bno_quat_w,"
        "bno_quat_x,"
        "bno_quat_y,"
        "bno_quat_z,"
        "compass_heading,"
        "flight_id\n"
    );
}


void flash::printPacketAsCSV(const FlightPacket& pkt) {
    // if (pkt.timestamp_utc == 0.0f) {
    //     return; // garbage packet, skip.
    // }
    pc->printf(
        "%.3f,"      // timestamp_timer
        "%.3f,"      // timestamp_utc
        "%u,"        // fsm_mode
        "%u,"        // gps_fix
        "%u,"        // gps_antenna_status
        "%.4f,"      // heading_deg
        "%.4f,"      // target_heading_deg
        "%.4f,"      // heading_error_deg
        "%.4f,"      // h_speed_m_s
        "%.4f,"      // v_speed_m_s
        "%.7f,"      // latitude_deg
        "%.7f,"      // longitude_deg
        "%.4f,"      // altitude_gps_m
        "%.4f,"      // altitude_bmp_m
        "%.4f,"      // altitude_m
        "%.4f,"      // pos_east_m
        "%.4f,"      // pos_north_m
        "%.4f,"      // distance_to_target_m
        "%.4f,"      // temp_c
        "%.4f,"      // pressure_pa
        "%.4f,"      // delta1_deg
        "%.4f,"      // delta1_m
        "%.4f,"      // delta2_deg
        "%.4f,"      // delta2_m
        "%.4f,"      // delta_a
        "%.4f,"      // delta_s
        "%.4f,"      // pwm_motor1
        "%.4f,"      // pwm_motor2
        "%.4f,"      // fc_cmd
        "%u,"        // apogee_counter
        "%u,"        // apogee_detected
        "%.4f,"      // yaw_rate
        "%.4f,"      // pitch_rate
        "%.4f,"      // roll_rate
        "%.4f,"      // bno_acc_x
        "%.4f,"      // bno_acc_y
        "%.4f,"      // bno_acc_z
        "%.4f,"      // bno_mag_x
        "%.4f,"      // bno_mag_y
        "%.4f,"      // bno_mag_z
        "%.4f,"      // bno_eul_x
        "%.4f,"      // bno_eul_y
        "%.4f,"      // bno_eul_z
        "%.4f,"      // bno_lin_x
        "%.4f,"      // bno_lin_y
        "%.4f,"      // bno_lin_z
        "%.4f,"      // bno_grav_x
        "%.4f,"      // bno_grav_y
        "%.4f,"      // bno_grav_z
        "%.4f,"      // bno_quat_w
        "%.4f,"      // bno_quat_x
        "%.4f,"      // bno_quat_y
        "%.4f,"      // bno_quat_z
        "%s,"        // compass_heading
        "%s\n",      // pkt.flight_id (uncomment if you add it back)
        pkt.timestamp_timer,
        pkt.timestamp_gps,
        pkt.fsm_mode,
        pkt.gps_fix,
        pkt.gps_antenna_status,
        pkt.heading_deg,
        pkt.target_heading_deg,
        pkt.heading_error_deg,
        pkt.h_speed_m_s,
        pkt.v_speed_m_s,
        pkt.latitude_deg,
        pkt.longitude_deg,
        pkt.altitude_gps_m,
        pkt.altitude_bmp_m,
        pkt.altitude_m,
        pkt.pos_east_m,
        pkt.pos_north_m,
        pkt.distance_to_target_m,
        pkt.temp_c,
        pkt.pressure_pa,
        pkt.delta_1_deg,
        pkt.delta_1_m,
        pkt.delta_2_deg,
        pkt.delta_2_m,
        pkt.delta_a,
        pkt.delta_s,
        pkt.pwm_motor1,
        pkt.pwm_motor2,
        pkt.fc_cmd,
        pkt.apogee_counter,
        static_cast<unsigned>(pkt.apogee_detected),
        pkt.yaw_rate,
        pkt.pitch_rate,
        pkt.roll_rate,
        pkt.bno_acc_x,
        pkt.bno_acc_y,
        pkt.bno_acc_z,
        pkt.bno_mag_x,
        pkt.bno_mag_y,
        pkt.bno_mag_z,
        pkt.bno_eul_x,
        pkt.bno_eul_y,
        pkt.bno_eul_z,
        pkt.bno_lin_x,
        pkt.bno_lin_y,
        pkt.bno_lin_z,
        pkt.bno_grav_x,
        pkt.bno_grav_y,
        pkt.bno_grav_z,
        pkt.bno_quat_w,
        pkt.bno_quat_x,
        pkt.bno_quat_y,
        pkt.bno_quat_z,
        pkt.compass_heading,
        pkt.flight_id
    );
}


void flash::dumpAllPackets(uint32_t numPackets) {
    FlightPacket pkt;
    printCSVHeader();
    for (uint32_t i = 0; i < numPackets; ++i) {
        readPacket(i * 256, pkt);
        // don't print if timestamp is nan
        // if (pkt.timestamp_utc == pkt.timestamp_utc) {
        printPacketAsCSV(pkt);
        // }
    }
}

/**
 * Converts a float value into a byte array (little endian).
 * @param ftoa_bytes_temp - Output byte array
 * @param float_variable - Input float value
 */
void float2Byte(uint8_t *ftoa_bytes_temp, float float_variable) {
    union {
        float f;
        uint8_t b[4];
    } conv;
    conv.f = float_variable;
    memcpy(ftoa_bytes_temp, conv.b, 4);
}

/**
 * Converts a byte array into a float value.
 * @param ftoa_bytes_temp - Input byte array (4 bytes)
 * @return Float reconstructed from byte array
 */
float bytes2float(uint8_t *ftoa_bytes_temp) {
    union {
        float f;
        uint8_t b[4];
    } conv;
    memcpy(conv.b, ftoa_bytes_temp, 4);
    return conv.f;
}

/* Implement this? (written by perplexity)...
bool verifyWrite(flash& fc, uint32_t addr, const FlightData& expected) {
    FlightData actual;
    fc.readPacket(addr, actual);
    return memcmp(&expected, &actual, sizeof(FlightData)) == 0;
}
*/