// bno055.h
#ifndef BNO055_SENSOR_H
#define BNO055_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class BNO055Sensor : public Sensor
{
public:
    BNO055Sensor();
    bool begin() override;
    void update() override;
    void setExternalCrystal();
    String getName() const override;
    String getData() const override;
    unsigned long getUpdateInterval() const override; // Implemented

    sensor_t sensor_val;

private:
    Adafruit_BNO055 sensor = Adafruit_BNO055(55, 0x28, &Wire);
    String sensorData_;
};

#endif // BNO055_SENSOR_H
