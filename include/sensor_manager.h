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
    void printAllData() const;

    void addSensor(std::shared_ptr<Sensor> sensor);

private:
    std::vector<std::shared_ptr<Sensor>> sensors;
};

#endif // SENSOR_MANAGER_H
