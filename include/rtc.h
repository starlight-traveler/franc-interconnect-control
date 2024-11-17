// RTC_Sensor.h
#ifndef RTC_SENSOR_H
#define RTC_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include <RTClib.h>

class RTCSensor : public Sensor
{
public:
    RTCSensor();
    bool begin() override;
    void update() override;
    String getName() const override;
    String getData() const override;
    unsigned long getUpdateInterval() const override; // Implemented

private:
    RTC_DS3231 rtc_;
    String sensorData_;
};

#endif // RTC_SENSOR_H
