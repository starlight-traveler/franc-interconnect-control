// bno085.cpp
#include "ens160.h"

ENS160Sensor::ENS160Sensor() : ens160(ENS160_I2CADDR_1) {}

bool ENS160Sensor::begin()
{
    // Serial.print("ENS160...");

    // Attempt to initialize the ENS160 sensor
    if (!ens160.begin())
    {
        // Serial.println("failed to initalize!");
        return false;
    }

    // Check if data is available after initialization
    // Serial.println(ens160.available() ? "done." : "failed!");

    if (ens160.available())
    {
        ens160.setMode(ENS160_OPMODE_STD);
    }

    return true;
}

void ENS160Sensor::update()
{
    // Check if the ENS160 sensor is available for a new measurement
    if (ens160.available())
    {
        // Perform standard measurement
        if (ens160.measure(true))
        {
            // Perform raw measurement
            if (ens160.measureRaw(true))
            {
                // Retrieve measurement data
                int aqi = ens160.getAQI();
                int tvoc = ens160.getTVOC(); // in ppb
                int eco2 = ens160.geteCO2(); // in ppm
                float hp0 = ens160.getHP0(); // Resistance in Ohms
                float hp1 = ens160.getHP1();
                float hp2 = ens160.getHP2();
                float hp3 = ens160.getHP3();

                // Format the sensor data as a CSV string
                // Example: "AQI,TVOC,eCO2,HP0,HP1,HP2,HP3"
                sensorData_ = String(aqi) + "," +
                              String(tvoc) + "," +
                              String(eco2) + "," +
                              String(hp0, 2) + "," +
                              String(hp1, 2) + "," +
                              String(hp2, 2) + "," +
                              String(hp3, 2);

                // Debugging: Print the sensor data to Serial Monitor
                // Serial.print("AQI: ");
                // Serial.print(aqi);
                // Serial.print("\tTVOC: ");
                // Serial.print(tvoc);
                // Serial.print(" ppb\t");
                // Serial.print("eCO2: ");
                // Serial.print(eco2);
                // Serial.print(" ppm\t");
                // Serial.print("R HP0: ");
                // Serial.print(hp0);
                // Serial.print(" Ohm\t");
                // Serial.print("R HP1: ");
                // Serial.print(hp1);
                // Serial.print(" Ohm\t");
                // Serial.print("R HP2: ");
                // Serial.print(hp2);
                // Serial.print(" Ohm\t");
                // Serial.print("R HP3: ");
                // Serial.println(hp3);
                // Serial.print(" Ohm");
            }
            else
            {
                Serial.println("ENS160 raw measurement failed.");
            }
        }
        else
        {
            Serial.println("ENS160 standard measurement failed.");
        }
    }
    else
    {
        Serial.println("ENS160 not ready for measurement.");
    }
}

String ENS160Sensor::getName() const
{
    return "ENS160";
}

String ENS160Sensor::getData() const
{
    return sensorData_;
}

unsigned long ENS160Sensor::getUpdateInterval() const
{
    return 1000; // Approximately 500 Hz (1000 ms / 500 ≈ 2 ms)
}
