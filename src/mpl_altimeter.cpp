// MPLAltimeterSensor.cpp
#include "mpl_altimeter.h"

MPLAltimeterSensor::MPLAltimeterSensor() : mpl() {}

bool MPLAltimeterSensor::begin()
{
    Wire.begin();
    if (!mpl.begin())
    {
        Serial.println("MPL3115A2 initialization failed!");
        return false;
    }

    setOversampleRate(MPL3115A2_CTRL_REG1_OS1);
    Serial.println("MPL3115A2 initialized.");
    mpl.setMode();
    
    return true;

}

void MPLAltimeterSensor::setOversampleRate(uint8_t oversampleRate)
{
    Wire.beginTransmission(MPL3115A2_ADDRESS);
    Wire.write(MPL3115A2_CTRL_REG1); // Register to write to
    Wire.write(oversampleRate);      // Oversample rate value
    Wire.endTransmission();

    Serial.print("Oversample rate set to: ");
    Serial.println(oversampleRate, HEX);
}

void MPLAltimeterSensor::update()
{
    // Implement timing check
    unsigned long currentTime = millis();
    unsigned long desiredInterval = getUpdateInterval();
    if (currentTime - lastUpdateTime >= desiredInterval)
    {
        float pressure = mpl.getPressure();
        float altitude = mpl.getAltitude();
        data = "Pressure: " + String(pressure) + " Pa, Altitude: " + String(altitude) + " m";
        lastUpdateTime = currentTime;
    }
}

String MPLAltimeterSensor::getName() const
{
    return "MPL3115A2";
}

String MPLAltimeterSensor::getData() const
{
    return data;
}

unsigned long MPLAltimeterSensor::getUpdateInterval() const
{
    return 40; // Approximately 30 Hz (1000 ms / 30 ≈ 33 ms)
}