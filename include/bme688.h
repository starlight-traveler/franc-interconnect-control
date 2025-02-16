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
    const SensorData *getData() const override;       // Updated return type
    unsigned long getUpdateInterval() const override; // Implemented

    bool hasNewData() const override;                 // Indicates if there's new data available
    void resetNewDataFlag() override;                 // Resets the new data flag
    void getData(void *data) const;                   // Retrieves the populated struct

    SensorType getSensorType() const override; // Implemented method

private:
    bool newDataFlag_;      // Tracks if new data is available
    BME688DataStruct data_; // Struct to hold sensor data
    Adafruit_BME680 sensor; 
    String sensorData_;
};

#endif // BME688_SENSOR_H
