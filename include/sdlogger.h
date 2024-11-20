// SDLogger.h
#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#define SD_LOG_FILE "datalog.bin"
#define WRITE_BUFFER_SIZE 10240 // 10 KB buffer

class SDLogger
{
public:
    SDLogger(uint8_t chipSelectPin);
    bool begin();
    void logMessage(uint8_t *data, size_t size);
    void flush(); // Force flush the buffer to SD card

private:
    uint8_t csPin_;
    uint8_t writeBuffer_[WRITE_BUFFER_SIZE];
    size_t bufferIndex_;
};

#endif // SD_LOGGER_H
