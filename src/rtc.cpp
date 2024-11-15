// RTC_Sensor.cpp
#include "rtc.h"

RTCSensor::RTCSensor() : rtc_() {}

bool RTCSensor::begin()
{
    Wire.begin();
    if (!rtc_.begin())
    {
        Serial.println("RTC_initialization_failed!");
        return false;
    }
    if (rtc_.lostPower())
    {
        Serial.println("RTC_lost_power,_setting_the_time!");
        rtc_.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.println("RTC_initialized.");
    return true;
}

void RTCSensor::update()
{
    DateTime now = rtc_.now();
    sensorData_ = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " +
                  String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
}

String RTCSensor::getName() const
{
    return "RTC";
}

String RTCSensor::getData() const
{
    return sensorData_;
}
