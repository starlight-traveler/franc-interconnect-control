// SensorManager.h
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <memory>
#include <vector>
#include "sensor.h"
#include "sdlogger.h"
#include "sensor.h"
#include "sensor_struct.h"
#include "master_sensor_struct.h"

#define CALC_INTERVAL_MS 1000 // 1-second interval for update rate calculation

class SensorManager
{
public:
    SensorManager(uint8_t sdChipSelectPin);
    bool begin();
    bool beginAll();
    void updateAll();
    void logAllData();
    void printAllData() const;
    void addSensor(std::shared_ptr<Sensor> sensor);
    float getUpdateRateHz() const;
    MasterSensorData getAggregatedData();

private:
    std::vector<std::shared_ptr<Sensor>> sensors;
    unsigned long lastUpdateTime_;
    unsigned long updateCount_;
    float updateRateHz_;
    bool headersWritten_;

    void logToCSV(unsigned long timestamp);
    SDLogger csvLogger_;

    // Latest data from each sensor
    BME688DataStruct latestBME688_;
    ENS160DataStruct latestENS160_;
    LSM6D032DataStruct latestLSM6D032_;
    MPLAltimeterDataStruct latestMPLAltimeter_;
    BNO055DataStruct latestBNO055_;
};

#endif // SENSOR_MANAGER_H