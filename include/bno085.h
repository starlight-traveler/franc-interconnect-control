// bno085.h
#ifndef BNO085_SENSOR_H
#define BNO085_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

class BNO085Sensor : public Sensor
{
public:
    BNO085Sensor();
    bool begin() override;
    void update() override;
    void setReports();
    String getName() const override;
    String getData() const override;
    unsigned long getUpdateInterval() const override; // Implemented

    sh2_SensorValue_t sensorValue;

private:
    Adafruit_BNO08x mySensor;
    String sensorData_;
};

#endif // BNO085_SENSOR_H
