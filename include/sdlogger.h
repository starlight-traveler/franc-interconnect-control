#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>
#include <SD.h>

#define WRITE_BUFFER_SIZE 512
#define SD_LOG_FILE_DEFAULT "log.bin" // Default binary log file

class SDLogger
{
public:
    SDLogger(uint8_t chipSelectPin, const char *logFileName = SD_LOG_FILE_DEFAULT);
    bool begin();
    void logMessage(uint8_t *data, size_t size);                     // For FlatBuffers
    void logCSV(const char *csvLine, const char *headers = nullptr); // For CSV logging
    void flush();                                                    // Flush buffer to SD card
    void fixIncompleteLineInBuffer();

    private : uint8_t csPin_;
    String fileName_;
    uint8_t writeBuffer_[WRITE_BUFFER_SIZE];
    size_t bufferIndex_;

    // Helper function to write buffer to file
    void writeToFile(const uint8_t *data, size_t size);

    // Helper function to check if file is empty
    bool isFileEmpty();
};

#endif // SDLOGGER_H
