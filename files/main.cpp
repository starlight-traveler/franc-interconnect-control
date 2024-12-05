#include <Arduino.h>
#include <SD.h>

const int chipSelect = 10;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing SD card...");

    pinMode(chipSelect, OUTPUT);

    if (!SD.begin(chipSelect))
    {
        Serial.println("Card failed, or not present");
        return;
    }
    Serial.println("Card initialized.");

    // Wait for a specific key from the serial port
    Serial.println("Waiting for key...");

    String key = "";
    while (key != "START")
    {
        if (Serial.available())
        {
            char c = Serial.read();
            if (c == '\n' || c == '\r')
            {
                // Ignore line endings
                continue;
            }
            key += c;
            // To prevent buffer overflow
            if (key.length() > 10)
            {
                key = ""; // Reset if key is too long
            }
        }
    }

    Serial.println("Key received. Dumping file...");

    // Open the file
    File dataFile = SD.open("log.csv");

    if (dataFile)
    {
        // Read data in chunks
        const size_t bufferSize = 64;
        uint8_t buffer[bufferSize];
        while (dataFile.available())
        {
            size_t bytesRead = dataFile.read(buffer, bufferSize);
            Serial.write(buffer, bytesRead);
            // Optional: Add a small delay
            // delay(1);
        }
        dataFile.close();
        Serial.println("\nFile transfer complete.");
    }
    else
    {
        Serial.println("Error opening log.csv");
    }
}

void loop()
{
    // Nothing to do here
}
