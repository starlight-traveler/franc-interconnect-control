#ifndef LSM6D032_SENSOR_H
#define LSM6D032_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>

class LSM6D032Sensor : public Sensor
{
public:
    LSM6D032Sensor();
    bool begin() override;
    void update() override;
    void setReports();
    String getName() const override;
    String getData() const override;
    unsigned long getUpdateInterval() const override;

private:
    Adafruit_LSM6DSO32 lsm;
    unsigned long lastUpdateTime;
    String sensorData_;

    
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
};

#endif 
