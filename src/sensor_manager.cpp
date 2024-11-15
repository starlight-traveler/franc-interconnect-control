// SensorManager.cpp
#include "sensor_manager.h"

SensorManager::SensorManager() {}

bool SensorManager::beginAll()
{
    bool allInitialized = true;
    for (auto &sensor : sensors)
    {
        if (!sensor->begin())
        {
            Serial.print(sensor->getName());
            Serial.println(" failed to initialize.");
            allInitialized = false;
        }
    }
    return allInitialized;
}

void SensorManager::updateAll()
{
    for (auto &sensor : sensors)
    {
        sensor->update();
    }
}

void SensorManager::printAllData() const
{
    for (const auto &sensor : sensors)
    {
        Serial.print(sensor->getName());
        Serial.print(": ");
        Serial.println(sensor->getData());
    }
}

void SensorManager::addSensor(std::shared_ptr<Sensor> sensor)
{
    sensors.push_back(sensor);
}
