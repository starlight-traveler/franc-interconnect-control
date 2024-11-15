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

private:
    Adafruit_MPL3115A2 mpl;
    String data;
};

#endif // MPL_ALTIMETER_SENSOR_H
