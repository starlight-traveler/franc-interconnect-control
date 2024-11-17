// MPLAltimeterSensor.h
#ifndef MPL_ALTIMETER_SENSOR_H
#define MPL_ALTIMETER_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

class MPLAltimeterSensor : public Sensor
{
public:
    MPLAltimeterSensor();
    bool begin() override;
    void update() override;
    String getName() const override;
    String getData() const override;
    void setOversampleRate(uint8_t oversampleRate);
    unsigned long getUpdateInterval() const override; // Implemented

private:
    Adafruit_MPL3115A2 mpl;
    uint8_t oversampleRate;
    unsigned long lastUpdateTime;
    String data;
};

#endif // MPL_ALTIMETER_SENSOR_H
