// SensorManager.cpp
#include "sensor_manager.h"

SensorManager::SensorManager()
    : lastUpdateTime_(0), updateCount_(0), updateRateHz_(0.0f) {}

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

void SensorManager::updateAllWithHz()
{
    for (auto &sensor : sensors)
    {
        sensor->update();
    }

    // Increment update count
    updateCount_++;

    // Get current time
    unsigned long currentTime = millis();

    // Check if the calculation interval has passed
    if (currentTime - lastUpdateTime_ >= CALC_INTERVAL_MS)
    {
        // Calculate update rate
        updateRateHz_ = (float)updateCount_ / ((currentTime - lastUpdateTime_) / 1000.0f);

        // Reset for the next interval
        lastUpdateTime_ = currentTime;
        updateCount_ = 0;
    }
}

float SensorManager::getUpdateRateHz() const
{
    return updateRateHz_;
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
