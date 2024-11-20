// ens160.h
#ifndef BME688_SENSOR_H
#define BME688_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define SEALEVELPRESSURE_HPA (1013.25)

class BME688Sensor : public Sensor
{
public:
    BME688Sensor();
    bool begin() override;
    void update() override;
    String getName() const override;
    String getData() const override;
    unsigned long getUpdateInterval() const override; // Implemented

private:
    Adafruit_BME680 sensor; 
    String sensorData_;
};

#endif // BME688_SENSOR_H
