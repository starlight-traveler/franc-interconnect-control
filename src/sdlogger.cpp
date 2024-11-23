#include "sdlogger.h"

SDLogger::SDLogger(uint8_t chipSelectPin, const char *logFileName)
    : csPin_(chipSelectPin), fileName_(logFileName), bufferIndex_(0) {}

bool SDLogger::begin()
{
    Serial.print("Initializing SD card for file: ");
    Serial.println(fileName_);

    // Set the CS pin as OUTPUT and deactivate the SD card
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH); // Deactivate SD card

    if (!SD.begin(csPin_))
    {
        Serial.println("SD initialization failed!");
        return false;
    }

    Serial.println("SD initialization done.");

    // Check if CSV file and write headers if needed
    if (fileName_.indexOf(".csv") != -1)
    {
        if (isFileEmpty())
        {
            // Headers should be provided when calling logCSV
            Serial.println("CSV file is empty. Headers need to be written.");
            // Headers will be written externally when logging the first line
        }
    }
    else if (fileName_.equals(SD_LOG_FILE_DEFAULT))
    {
        // Optionally, clear the log file at start if it's a binary log
        File dataFile = SD.open(fileName_.c_str(), FILE_WRITE);
        if (dataFile)
        {
            dataFile.close(); // Clear contents
            Serial.println("Log file cleared.");
        }
        else
        {
            Serial.print("Failed to open log file: ");
            Serial.println(fileName_);
            return false;
        }
    }

    return true;
}

bool SDLogger::isFileEmpty()
{
    File file = SD.open(fileName_.c_str(), FILE_READ);
    if (file)
    {
        size_t size = file.size();
        file.close();
        return size == 0;
    }
    // If file doesn't exist, it's considered empty
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
        writeToFile(data, size);
        Serial.println("Large message written directly.");
        return;
    }

    memcpy(writeBuffer_ + bufferIndex_, data, size);
    bufferIndex_ += size;
}

void SDLogger::logCSV(const char *csvLine, const char *headers)
{
    // If headers are provided and file is empty, write headers first
    if (headers != nullptr && isFileEmpty())
    {
        // Append newline to headers
        String headerStr = String(headers) + "\n";
        logCSV(headerStr.c_str()); // Recursive call to write headers
    }

    size_t len = strlen(csvLine);
    if (bufferIndex_ + len > WRITE_BUFFER_SIZE)
    {
        flush(); // Flush if buffer is full
    }

    if (bufferIndex_ + len > WRITE_BUFFER_SIZE)
    {
        // If single line is larger than buffer, write it directly
        writeToFile((const uint8_t *)csvLine, len);
        Serial.println("Large CSV line written directly.");
        return;
    }

    memcpy(writeBuffer_ + bufferIndex_, csvLine, len);
    bufferIndex_ += len;
}

void SDLogger::flush()
{
    if (bufferIndex_ == 0)
        return; // Nothing to flush

    writeToFile(writeBuffer_, bufferIndex_);
    bufferIndex_ = 0;
}

void SDLogger::writeToFile(const uint8_t *data, size_t size)
{
    File dataFile = SD.open(fileName_.c_str(), FILE_WRITE);
    if (dataFile)
    {
        dataFile.write(data, size);
        dataFile.close();
        // Serial.println("Wrote file...");
    }
    else
    {
        // Serial.print("Failed to open log file for writing: ");
        // Serial.println(fileName_);
    }
}
