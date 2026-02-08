#include "I2CSerial.h"
#include <cstring>


I2CSerial::I2CSerial(PinName SDA, PinName SCL, int address, bool slave) {
    this->own_i2c = true;

    this->is_slave = slave;
    this->address = address;

    if (is_slave) {
        this->slave = new I2CSlave(SDA, SCL);
        this->slave->address(address);
    }
    else {
        this->i2c = new I2C(SDA, SCL);
    }

    // clear buffers and pointers
    msgs_pointer = 0;
    memset(msgs_buf, 0, MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE);
    write_pointer = 0;
    memset(write_buf, 0, MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE);

    this->running = true;
    t.set_priority(osPriorityBelowNormal1);
    t.start([this]() {loop();});
}

I2CSerial::I2CSerial(I2C* i2c, int address) {
    this->own_i2c = false;
    this->i2c = i2c;
    this->is_slave = false;
    this->address = address;

    // clear buffers and pointers
    msgs_pointer = 0;
    memset(msgs_buf, 0, MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE);
    write_pointer = 0;
    memset(write_buf, 0, MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE);

    this->running = true;
    t.set_priority(osPriorityBelowNormal1);
    t.start([this]() {loop();});
}

I2CSerial::~I2CSerial() {
    if (!own_i2c) {
        delete this->i2c;
    }

    if (is_slave) {
        delete this->slave;
    }

    this->running = false;
    t.join();
}

void I2CSerial::loop() {
    // slave code, listen for i2c events and respond as requested
    if (is_slave) {
        while (true) {
            int i = slave->receive();
            switch (i) {
                case I2CSlave::ReadAddressed:
                    if (write_pointer > 0) {
                        slave->write(write_buf, MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE);
                        write_pointer = 0;
                    }
                    break;
                case I2CSlave::WriteGeneral:
                    break;
                case I2CSlave::WriteAddressed:
                    slave->read(&msgs_buf[msgs_pointer], MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE);
                    size_t len = strlen(&msgs_buf[msgs_pointer]);
                    msgs_pointer += (len > 0) ? len+1 : len;

                    break;
            }
            ThisThread::yield();
        }
    }

    // master code polls slave for new messages
    else {
        while (true) {
            i2c->read(this->address, &msgs_buf[msgs_pointer], MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE);
            size_t len = strlen(&msgs_buf[msgs_pointer]);
            msgs_pointer += (len > 0) ? len+1 : len;

            if (write_pointer > 0) {
                i2c->write(this->address, write_buf, MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE);
                write_pointer = 0;
            }

            ThisThread::sleep_for(MBED_CONF_I2CSERIAL_MSG_POLL_MS);
        }
    }
}

I2CSerialError I2CSerial::printf(const char* format, ...) {
    va_list args;

    char buf[MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE] = {0};

    va_start(args, format);

    size_t n = vsnprintf(buf, MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE-write_pointer, format, args);
    n += 1; // null terminator
    
    va_end(args);

    if (n > MBED_CONF_I2CSERIAL_MAX_PACKET_SIZE) {
        return I2CSerialError::I2CS_MSG_TOO_LARGE;
    }

    return write(buf, n);
}

I2CSerialError I2CSerial::write(const char* buf, size_t size) {
    if (size > MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE-write_pointer) {
        return I2CSerialError::I2CS_WRITE_BUFFER_FULL;
    }

    memcpy(&write_buf[write_pointer], buf, size);
    write_pointer += size;

    return I2CSerialError::I2CS_OK;
}

bool I2CSerial::read(char* buf, size_t size) {
    if (msgs_pointer < size) {
        return false;
    }

    size = (size > MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE) ? MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE : size;

    memcpy(buf, msgs_buf, size);
    for (int i = size; i<MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE; i++) {
        msgs_buf[i-size] = msgs_buf[i];
        msgs_buf[i] = 0;
    }
    msgs_pointer -= size;

    return true;
}

bool I2CSerial::readline(char* buf, size_t size) {
    if (msgs_pointer == 0) {
        return false;
    }

    bool success = false;


    int len = 0;

    for (int i = 0; i<size; i++) {
        if (msgs_buf[i] == '\n') {
            len = i+2;
            memcpy(buf, msgs_buf, len);
            success = true;
            break;
        }
        if (msgs_buf[i] == 0) {
            return false;
        }
    }

    if (success) {
        for (int i = len; i<MBED_CONF_I2CSERIAL_MSG_BUFFER_SIZE; i++) {
            msgs_buf[i-len] = msgs_buf[i];
            msgs_buf[i] = 0;
        }
        msgs_pointer -= len;
    }

    return success;
}
