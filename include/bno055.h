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
    const SensorData *getData() const override;       // Updated return type
    unsigned long getUpdateInterval() const override; // Implemented

    bool hasNewData() const override; // Indicates if there's new data available
    void resetNewDataFlag() override; // Resets the new data flag
    void getData(void *data) const;   // Retrieves the populated struct
    SensorType getSensorType() const override; // Implemented method

    sensor_t sensor_val;

private:
    Adafruit_BNO055 sensor = Adafruit_BNO055(55, 0x28, &Wire);
    BNO055DataStruct data_; // Struct to hold sensor data
    bool newDataFlag_;      // Tracks if new data is available
    String sensorData_;
};

#endif // BNO055_SENSOR_H
