// bno085.cpp
#include "bme688.h"

BME688Sensor::BME688Sensor() : sensor(&Wire) {}

bool BME688Sensor::begin()
{
    // Try to initialize!
    if (!sensor.begin())
    {
        // Serial.println("Could not find a valid BME680 sensor, check wiring!");

        return false;
    }

    sensor.setTemperatureOversampling(BME680_OS_8X);
    sensor.setHumidityOversampling(BME680_OS_2X);
    sensor.setPressureOversampling(BME680_OS_4X);
    sensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
    sensor.setGasHeater(320, 150); // 320*C for 150 ms

    return true;

}

void BME688Sensor::update()
{
    // Check if a new measurement is available
    if (!sensor.performReading())
    {
        Serial.println("Failed to perform reading :(");
        return;
    }

    // Retrieve sensor data
    float temperature = sensor.temperature;                     // in Celsius
    float pressure = sensor.pressure / 100.0;                   // in hPa
    float humidity = sensor.humidity;                           // in %
    float gas = sensor.gas_resistance / 1000.0;                 // in KOhms
    float altitude = sensor.readAltitude(SEALEVELPRESSURE_HPA); // in meters

    // Format the sensor data as a CSV string
    // Example: "Temperature,Pressure,Humidity,Gas,Altitude"
    sensorData_ = String(temperature, 2) + "," +
                  String(pressure, 2) + "," +
                  String(humidity, 2) + "," +
                  String(gas, 2) + "," +
                  String(altitude, 2);

    // // Debugging: Print the sensor data to Serial Monitor
    // Serial.print("Temperature = ");
    // Serial.print(temperature);
    // Serial.println(" *C");

    // Serial.print("Pressure = ");
    // Serial.print(pressure);
    // Serial.println(" hPa");

    // Serial.print("Humidity = ");
    // Serial.print(humidity);
    // Serial.println(" %");

    // Serial.print("Gas = ");
    // Serial.print(gas);
    // Serial.println(" KOhms");

    // Serial.print("Approx. Altitude = ");
    // Serial.print(altitude);
    // Serial.println(" m");

    // Serial.println(); // Blank line for readability
}

String BME688Sensor::getName() const
{
    return "BME688";
}

String BME688Sensor::getData() const
{
    return sensorData_;
}

unsigned long BME688Sensor::getUpdateInterval() const
{
    return 1; // Approximately 500 Hz (1000 ms / 500 ≈ 2 ms)
}
