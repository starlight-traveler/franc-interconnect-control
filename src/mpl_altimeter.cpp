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
    Serial.println("MPL3115A2 initialized.");
    mpl.setMode();
    return true;
}

void MPLAltimeterSensor::update()
{
    float pressure = mpl.getPressure();
    float altitude = mpl.getAltitude();
    data = "Pressure: " + String(pressure) + " Pa, Altitude: " + String(altitude) + " m";
}

String MPLAltimeterSensor::getName() const
{
    return "MPL3115A2";
}

String MPLAltimeterSensor::getData() const
{
    return data;
}
