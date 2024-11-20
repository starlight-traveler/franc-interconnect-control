// MPLAltimeterSensor.cpp
#include "mpl_altimeter.h"

MPLAltimeterSensor::MPLAltimeterSensor() : mpl(), newDataFlag_(false), lastUpdateTime(0) {}

bool MPLAltimeterSensor::begin()
{
    Wire.begin();
    if (!mpl.begin())
    {
        return false;
    }

    setOversampleRate(MPL3115A2_CTRL_REG1_OS1);
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
    unsigned long currentTime = millis();
    unsigned long desiredInterval = getUpdateInterval();
    if (currentTime - lastUpdateTime >= desiredInterval)
    {
        // Retrieve data from the sensor
        data_.pressure = mpl.getPressure(); // Populate pressure in Pa
        data_.altitude = mpl.getAltitude(); // Populate altitude in meters

        // Set new data flag
        newDataFlag_ = true;

        // Update the last update time
        lastUpdateTime = currentTime;
    }
}

String MPLAltimeterSensor::getName() const
{
    return "MPL3115A2";
}

String MPLAltimeterSensor::getData() const
{
    // Optionally convert struct data to a readable string
    return "Pressure: " + String(data_.pressure) + " Pa, Altitude: " + String(data_.altitude) + " m";
}

unsigned long MPLAltimeterSensor::getUpdateInterval() const
{
    return 40; // Approximately 30 Hz (1000 ms / 30 ≈ 33 ms)
}

bool MPLAltimeterSensor::hasNewData() const
{
    return newDataFlag_;
}

void MPLAltimeterSensor::resetNewDataFlag()
{
    newDataFlag_ = false;
}

void MPLAltimeterSensor::getData(void *data) const
{
    if (data)
    {
        memcpy(data, &data_, sizeof(MPLAltimeterDataStruct));
    }
}