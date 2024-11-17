// SensorManager.h
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "sensor.h"
#include <vector>
#include <memory>

class SensorManager
{
public:
    SensorManager();
    bool beginAll();
    void updateAll();
    void updateAllWithHz();
    void printAllData() const;

    void addSensor(std::shared_ptr<Sensor> sensor);
    float getUpdateRateHz() const;

private:
    std::vector<std::shared_ptr<Sensor>> sensors;
    // Variables for update rate calculation
    unsigned long lastUpdateTime_; // Time of the last update rate calculation
    unsigned int updateCount_;     // Number of updates since last calculation
    float updateRateHz_;           // Calculated update rate in Hz

    static const unsigned long CALC_INTERVAL_MS = 1000; // Calculation interval (e.g., 1000ms = 1s)

};

#endif // SENSOR_MANAGER_H
