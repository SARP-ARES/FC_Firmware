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
    mutex.lock(); //Mutex i2c transfer
    i2c.write(addr, &regAddr, 1);
    uint8_t status = i2c.read(addr, data, len);
    mutex.unlock();
    return status; 
}

/** 
 * @brief Writes data to a BNO055 register over I2C.
 * @param addr The I2C address to write to
 * @param regAddr The register address to write to
 * @param data The value to be written
 * @return 0 on success, non-zero on failure
 */
int Mutex_I2C::writeData(char addr, char regAddr, char data) {
    mutex.lock();
    char buffer[2];
    buffer[0] = regAddr;
    buffer[1] = data;
    uint8_t status = i2c.write(addr, buffer, 2);
    mutex.unlock();
    return status; 
}