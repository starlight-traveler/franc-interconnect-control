// ens160.h
#ifndef ENS160_SENSOR_H
#define ENS160_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include "ScioSense_ENS160.h" // ENS160 library

class ENS160Sensor : public Sensor
{
public:
    ENS160Sensor();
    bool begin() override;
    void update() override;
    String getName() const override;
    String getData() const override;
    unsigned long getUpdateInterval() const override; // Implemented

private:
    ScioSense_ENS160 ens160; // Declaration only

    String sensorData_;
};

#endif // ENS160_SENSOR_H
