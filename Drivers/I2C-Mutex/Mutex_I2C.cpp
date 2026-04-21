#include "mbed.h"
#include "Mutex_I2C.h"
using namespace std;

Mutex_I2C::Mutex_I2C(PinName SDA, PinName SCL) : i2c(SDA, SCL) {
    owned = true; 
}

/**
 * @brief Reads a chunk of data over I2C
 * @param addr the address to read from
 * @param regAddr The register address to read from
 * @param data Pointer to a buffer for storing data
 * @param len Number of bytes to read
 * @return 0 on success, non-zero on failure
 */
int Mutex_I2C::readData(char addr, char regAddr, char* data, uint8_t len) {
    this->write(addr, &regAddr, 1);
    return this->read(addr, data, len);
}

/** 
 * @brief Writes data to a BNO055 register over I2C.
 * @param addr The I2C address to write to
 * @param regAddr The register address to write to
 * @param data The value to be written
 * @return 0 on success, non-zero on failure
 */
int Mutex_I2C::writeData(char addr, char regAddr, char data) {
    char buffer[2];
    buffer[0] = regAddr;
    buffer[1] = data;
    return this->write(addr, buffer, 2);
}

int Mutex_I2C::write(char addr, const char *data, int length) { 
    mutex.lock();
    _flags.clear(I2C_COMPLETE);

    int err = i2c.transfer(
        addr,
        data, length, 
        nullptr, 0,
        callback(this, &Mutex_I2C::_i2c_callback),
        I2C_EVENT_ALL
    );


    if( err != 0 ){ // Failed state
        mutex.unlock();
        return err;
    }

    
    uint32_t result = _flags.wait_any(I2C_COMPLETE, I2C_TIMEOUT_MS);

    int status = 0;
    if (result == osFlagsErrorTimeout){
        i2c.abort_transfer();
        status = -2; // timeout failure
    }

    mutex.unlock();
    return status;
}

int Mutex_I2C::read(char addr, char *data, int length) {
    mutex.lock();
    _flags.clear(I2C_COMPLETE);

    // Non blocking transfer
    int err = i2c.transfer(
        addr,
        nullptr, 0, // Write params
        data, length,
        callback(this, &Mutex_I2C::_i2c_callback),
        I2C_EVENT_ALL
    );
    
    if( err != 0 ){ // Failed state
        mutex.unlock();
        return err;
    }

    // Wait for I2C complete flag to be set
    uint32_t result = _flags.wait_any(I2C_COMPLETE, I2C_TIMEOUT_MS);

    int status = 0;
    if (result == osFlagsErrorTimeout){
        i2c.abort_transfer();
        status = -2; // timeout failure
    }

    mutex.unlock(); 
    return status; 
}