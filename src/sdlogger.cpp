// sdlogger.cpp
#include "sdlogger.h"

SDLogger::SDLogger(uint8_t chipSelectPin)
    : csPin_(chipSelectPin), bufferIndex_(0) {}

bool SDLogger::begin()
{
    Serial.print("Initializing SD card...");
    if (!SD.begin(csPin_))
    {
        Serial.println("Initialization failed!");
        return false;
    }
    Serial.println("Initialization done.");

    // Optionally, clear the log file at start
    File dataFile = SD.open(SD_LOG_FILE, FILE_WRITE);
    if (dataFile)
    {
        dataFile.close(); // Close to clear contents
        Serial.println("Log file cleared.");
    }
    else
    {
        Serial.println("Failed to open log file.");
        return false;
    }

    return true;
}

void SDLogger::logMessage(uint8_t *data, size_t size)
{
    if (bufferIndex_ + size > WRITE_BUFFER_SIZE)
    {
        flush(); // Flush if buffer is full
    }

    if (bufferIndex_ + size > WRITE_BUFFER_SIZE)
    {
        // If single message is larger than buffer, write it directly
        File dataFile = SD.open(SD_LOG_FILE, FILE_WRITE);
        if (dataFile)
        {
            dataFile.write(data, size);
            dataFile.close();
            Serial.println("Large message written directly.");
        }
        else
        {
            Serial.println("Failed to open log file for large message.");
        }
        return;
    }

    memcpy(writeBuffer_ + bufferIndex_, data, size);
    bufferIndex_ += size;
}

void SDLogger::flush()
{
    if (bufferIndex_ == 0)
        return; // Nothing to flush

    File dataFile = SD.open(SD_LOG_FILE, FILE_WRITE);
    if (dataFile)
    {
        dataFile.write(writeBuffer_, bufferIndex_);
        dataFile.close();
        Serial.println("Buffer flushed to SD card.");
        bufferIndex_ = 0;
    }
    else
    {
        Serial.println("Failed to open log file for flushing.");
    }
}
