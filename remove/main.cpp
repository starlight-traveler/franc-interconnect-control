#include <Arduino.h>
#include <SD.h>

// Chip select pin for the SD card module
const int chipSelect = 10;

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(9600);
    while (!Serial)
    {
        ; // Wait for serial port to connect
    }

    Serial.println("Initializing SD card...");

    // Set the chip select pin as output
    pinMode(chipSelect, OUTPUT);

    // Initialize the SD card
    if (!SD.begin(chipSelect))
    {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized.");

    // Specify the file to delete
    const char *filename = "logger.csv";

    // Check if the file exists
    if (SD.exists(filename))
    {
        Serial.print("Deleting file: ");
        Serial.println(filename);

        // Delete the file
        if (SD.remove(filename))
        {
            Serial.println("File deleted successfully.");
        }
        else
        {
            Serial.println("Failed to delete the file.");
        }
    }
    else
    {
        Serial.print("File does not exist: ");
        Serial.println(filename);
    }
}

void loop()
{
    // Nothing to do here
}
