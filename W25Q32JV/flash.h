#ifndef FLASH_H
#define FLASH_H
#include "flight_packet.h" // contains FlightData struct definition
#include "mbed.h"
#include "EUSBSerial.h"

const uint32_t STORAGE_BYTES = 2097152;
const uint32_t PACKET_SIZE = sizeof(FlightPacket);
const uint32_t MAX_PACKETS = floor(STORAGE_BYTES / PACKET_SIZE);

class flash {
public:
    // Constructor
    flash(PinName mosi, PinName miso, PinName sclk, PinName csPin);

    // Constructs flash chip with pointer to EUSBSerial object for printing 
    flash(PinName mosi, PinName miso, PinName sclk, PinName csPin, EUSBSerial* pc);

    // Read operations
    void read(uint32_t address, uint8_t *buffer, size_t length);
    uint8_t readByte(uint32_t address);
    float readNum(uint32_t address);
    uint32_t readPacket(uint32_t address, FlightPacket& pkt);

    // Read data to CSV
    void printCSVHeader();
    void printPacketAsCSV(const FlightPacket& pkt);
    void dumpAllPackets(uint32_t numPackets);

    // Write operations
    uint32_t write(uint32_t address, const uint8_t *buffer, size_t length);
    void writeByte(uint32_t address, uint8_t data);
    uint32_t writeNum(uint32_t address, float data);
    uint32_t writePacket(uint32_t address, const FlightPacket& pkt);
    

    // Erase operations
    void eraseSector(uint32_t address);
    int eraseAll();

    // Control operations
    void enableWrite();
    void disableWrite();
    void reset();

private:
    SPI _spi;       // SPI communication interface
    DigitalOut _cs; // Chip Select (CS) pin
    EUSBSerial* pc;
    Mutex lock;


    // Helper functions for SPI communication
    void csLow();
    void csHigh();
};

// Float conversion functions
void float2Byte(uint8_t *ftoa_bytes_temp, float float_variable);
float bytes2float(uint8_t *ftoa_bytes_temp);

#endif // FLASH_H
