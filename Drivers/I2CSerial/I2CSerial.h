/* I2CSerial
 * Ethan Armstrong
 * warmst@uw.edu
 * -------------------------------------------
 * I2C Serial is a class that allows for the sending
 * and recieving of UART style meessages over an I2C
 * interface
 * 
 * Messages are stored in a TX and RX Buffer, which
 * can be size adjusted by setting the following
 * mbed config options:
 *     I2CSERIAL_MAX_PACKET_SIZE : 256  // TX buffer
 *     I2CSERIAL_MSG_BUFFER_SIZE : 1024 // RX buffer
 *
 * The master will run a seperate thread that will
 * buffer each message as it comes in. Each cycle
 * This thread will move MAX_PACKET_SIZE bytes from
 * the slave into the RX buffer, and will send
 * MAX_PACKET_SIZE bytes to the slave.
 * The period this thread runs at can be modified
 * with the following mbed config option:
 *     I2CSERIAL_MSG_POLL_MS : 100
 */

#include "mbed.h"

#ifndef MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE
#define MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE 256
#endif

#ifndef MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE
#define MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE 1024
#endif

#ifndef MBED_CONF_I2CSERIAL_MSG_POLL_MS
#define MBED_CONF_I2CSERIAL_MSG_POLL_MS 100
#endif

#ifndef _I2C_SERIAL_H_
#define _I2C_SERIAL_H_

typedef enum {
    I2CS_OK,
    I2CS_NO_MESSAGES,
    I2CS_MSG_TOO_LARGE,
    I2CS_WRITE_BUFFER_FULL,
} I2CSerialError;

class I2CSerial {
public:
    /* I2CSerial() | creates a new I2CSerial master / slave Object
     *     SDA (PinName)  | I2C SDA Pin
     *     SCL (PinName)  | I2C SCL Pin
     *     address (int)  | I2C address
     *     slave   (bool) | is device the master or slave
    */
    I2CSerial(PinName SDA, PinName SCL, int address, bool slave = false);

    /* I2CSerial() | creates a new I2CSerial master Object
     *     i2c (I2C*)     | I2C object
     *     address (int)  | I2C address
     *     slave   (bool) | is device the master or slave
    */
    I2CSerial(I2C* i2c, int address);
    ~I2CSerial();

    /* printf() | formatted print
     *     format (const char *) | format string
     *     ...  (*)              | arguments for format string
     *
     * returns I2CSerialError
    */
    I2CSerialError printf(const char* format, ...);

    /* write() | writes entire buffer
     *     buf (const char *) | buffer to send (zero terminated)
     *     size (size_t)      | buffer size
     *
     * returns I2CSerialError
    */
    I2CSerialError write(const char* buf, size_t size);

    /* read() | reads to fill the entire buffer
     *     buf (const char *) | buffer to read to
     *     size (size_t)      | buffer size
     *
     * returns false if no data is available, true otherwise
    */
    bool read(char* buf, size_t size);

    /* readline() | reads a \n terminated string into a buffer
     *     buf (const char *) | buffer to read to
     *     size (size_t)      | buffer size
     *
     * returns false if no \n terminated strings are found, true otherwise
    */
    bool readline(char* buf, size_t size);

private:
    char msgs_buf[MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE];
    char write_buf[MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE];
    size_t msgs_pointer;
    size_t write_pointer;
    bool is_slave;

    bool own_i2c;
    

    I2C* i2c;
    I2CSlave* slave;

    int address;

    bool running;
    Thread t;

    // background thread
    void loop();
};

#endif //_I2C_SERIAL_H_