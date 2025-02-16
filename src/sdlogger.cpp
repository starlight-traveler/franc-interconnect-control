#include "sdlogger.h"
#include <Arduino.h>
#include <SD.h>
#include <string.h>

#define EXPECTED_FIELD_COUNT 44 // Total number of CSV fields (header should have 44 fields)
                                // Thus, there should be EXPECTED_FIELD_COUNT - 1 commas per line

// Helper: Count commas in a string
static int countCommas(const char *str)
{
    int count = 0;
    for (size_t i = 0; str[i] != '\0'; i++)
    {
        if (str[i] == ',')
            count++;
    }
    return count;
}

SDLogger::SDLogger(uint8_t chipSelectPin, const char *logFileName)
    : csPin_(chipSelectPin), fileName_(logFileName), bufferIndex_(0)
{
}

bool SDLogger::begin()
{
    // Serial.print("Initializing SD card for file: ");
    Serial.println(fileName_);

    // Set the CS pin as OUTPUT and deactivate the SD card
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH); // Deactivate SD card

    if (!SD.begin(csPin_))
    {
        Serial.println("SD initialization failed!");
        return false;
    }

    // Serial.println("SD initialization done.");

    // Check if this is a CSV file
    if (fileName_.indexOf(".csv") != -1)
    {
        if (isFileEmpty())
        {
            Serial.println("CSV file is empty. Headers need to be written.");
            // Headers will be written when logCSV() is called with a non-null headers parameter.
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

//
// fixIncompleteLineInBuffer()
// Checks the writeBuffer_ (from the last newline to the current end) for an incomplete CSV line.
// If the last line does not end with a newline or has too few fields, it appends the missing fields (",0")
// and a newline, as long as there is room in the buffer.
//
void SDLogger::fixIncompleteLineInBuffer()
{
    if (bufferIndex_ == 0)
        return; // Nothing to fix

    // Find the index of the last newline in the current buffer
    int lastNewline = -1;
    for (int i = bufferIndex_ - 1; i >= 0; i--)
    {
        if (writeBuffer_[i] == '\n')
        {
            lastNewline = i;
            break;
        }
    }

    // If the buffer ends with a newline, the last line is complete.
    if (writeBuffer_[bufferIndex_ - 1] == '\n')
        return;

    // Otherwise, the incomplete line starts from lastNewline+1 (or 0 if no newline found)
    int start = (lastNewline == -1) ? 0 : lastNewline + 1;
    int len = bufferIndex_ - start;
    char tempLine[1024];
    if (len >= (int)sizeof(tempLine))
        len = sizeof(tempLine) - 1;
    memcpy(tempLine, writeBuffer_ + start, len);
    tempLine[len] = '\0';

    // Count the number of commas
    int commaCount = countCommas(tempLine);
    int expectedCommaCount = EXPECTED_FIELD_COUNT - 1;
    if (commaCount < expectedCommaCount)
    {
        // Build string to append missing fields (",0" for each missing field)
        int missing = expectedCommaCount - commaCount;
        char missingStr[128] = "";
        for (int i = 0; i < missing; i++)
        {
            strncat(missingStr, ",0", sizeof(missingStr) - strlen(missingStr) - 1);
        }
        size_t missingLen = strlen(missingStr);
        // Append missing fields if room permits
        if (bufferIndex_ + missingLen + 1 < WRITE_BUFFER_SIZE)
        {
            memcpy(writeBuffer_ + bufferIndex_, missingStr, missingLen);
            bufferIndex_ += missingLen;
            // Append newline
            writeBuffer_[bufferIndex_] = '\n';
            bufferIndex_++;
            Serial.print("Fixed incomplete line by adding ");
            Serial.print(missing);
            Serial.println(" missing field(s).");
        }
        else
        {
            Serial.println("Error: Not enough space in buffer to fix incomplete line.");
        }
    }
    else
    {
        // If the line has the expected number of fields but is missing a newline, append it.
        if (bufferIndex_ < WRITE_BUFFER_SIZE)
        {
            writeBuffer_[bufferIndex_] = '\n';
            bufferIndex_++;
        }
    }
}

//
// logCSV()
// Now calls fixIncompleteLineInBuffer() before appending a new line.
// Also, it performs CSV correction on the incoming line as before.
//
void SDLogger::logCSV(const char *csvLine, const char *headers)
{
    // If headers are provided and the file is empty, write headers first.
    if (headers != nullptr && isFileEmpty())
    {
        // Ensure the header ends with a newline.
        String headerStr = String(headers);
        if (!headerStr.endsWith("\n"))
        {
            headerStr += "\n";
        }
        size_t headerLen = headerStr.length();
        if (bufferIndex_ + headerLen > WRITE_BUFFER_SIZE)
        {
            flush();
        }
        if (bufferIndex_ + headerLen > WRITE_BUFFER_SIZE)
        {
            // Write header directly if it exceeds the buffer size.
            writeToFile((const uint8_t *)headerStr.c_str(), headerLen);
            Serial.println("Large header line written directly.");
        }
        else
        {
            memcpy(writeBuffer_ + bufferIndex_, headerStr.c_str(), headerLen);
            bufferIndex_ += headerLen;
        }
        flush(); // Force flush so the header is actually written to the file.
    }

    // ----- First, fix any incomplete line already in the buffer -----
    fixIncompleteLineInBuffer();

    // ----- Process the incoming CSV line to correct missing fields -----
    char correctedLine[1024];
    // Copy the incoming line
    strncpy(correctedLine, csvLine, sizeof(correctedLine));
    correctedLine[sizeof(correctedLine) - 1] = '\0';

    // Ensure the line ends with a newline.
    size_t len = strlen(correctedLine);
    if (len == 0 || correctedLine[len - 1] != '\n')
    {
        strncat(correctedLine, "\n", sizeof(correctedLine) - strlen(correctedLine) - 1);
    }

    // Count commas in the corrected line.
    int commaCount = countCommas(correctedLine);
    int expectedCommaCount = EXPECTED_FIELD_COUNT - 1;
    if (commaCount < expectedCommaCount)
    {
        // Append missing fields for this line.
        int missing = expectedCommaCount - commaCount;
        for (int i = 0; i < missing; i++)
        {
            strncat(correctedLine, ",0", sizeof(correctedLine) - strlen(correctedLine) - 1);
        }
        // Ensure newline at end.
        if (correctedLine[strlen(correctedLine) - 1] != '\n')
        {
            strncat(correctedLine, "\n", sizeof(correctedLine) - strlen(correctedLine) - 1);
        }
        Serial.print("Warning: Corrected CSV line by adding ");
        Serial.print(missing);
        Serial.println(" missing field(s).");
    }
    else if (commaCount > expectedCommaCount)
    {
        Serial.println("Warning: CSV line has too many fields.");
    }
    // ----- End CSV correction -----

    // Now write the corrected CSV line.
    size_t correctedLen = strlen(correctedLine);
    if (bufferIndex_ + correctedLen > WRITE_BUFFER_SIZE)
    {
        flush();
    }
    if (bufferIndex_ + correctedLen > WRITE_BUFFER_SIZE)
    {
        // Write the CSV line directly if it exceeds the buffer.
        writeToFile((const uint8_t *)correctedLine, correctedLen);
        Serial.println("Large CSV line written directly.");
        return;
    }
    memcpy(writeBuffer_ + bufferIndex_, correctedLine, correctedLen);
    bufferIndex_ += correctedLen;
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
    }
    else
    {
        Serial.print("Failed to open log file for writing: ");
        Serial.println(fileName_);
    }
}
