// sensor_manager
#include "sensor_manager.h"
#include "mpl_altimeter.h"
#include "sensor_struct.h"

SensorManager::SensorManager(uint8_t sdChipSelectPin)
    : lastUpdateTime_(0), updateCount_(0), updateRateHz_(0.0f),
      sdLogger_(sdChipSelectPin), builder_(1024), // Initialize builder with 1KB buffer
      latestBME688_(), latestENS160_(), latestLSM6D032_(),
      latestMPLAltimeter_(), latestBNO055_()
{
}

bool SensorManager::beginAll()
{
    if (!sdLogger_.begin())
    {
        Serial.println("SDLogger failed to initialize.");
        return false;
    }

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

        Serial.print("Update Rate: ");
        Serial.print(updateRateHz_);
        Serial.println(" Hz");
    }
}

void SensorManager::logAllData()
{
    unsigned long timestamp = millis();

    // Update only MPLAltimeter data with real values
    for (auto &sensor : sensors)
    {
        if (sensor->hasNewData())
        {
            if (auto mpl = std::static_pointer_cast<MPLAltimeterSensor>(sensor))
            {
                mpl->getData(&latestMPLAltimeter_);
                sensor->resetNewDataFlag();
            }
        }
    }

    // Create FlatBuffers for other sensors with default values (0)
    auto bme688Data = SensorLog::CreateBME688Data(builder_, 0, 0, 0, 0, 0);
    auto ens160Data = SensorLog::CreateENS160Data(builder_, 0, 0, 0, 0.0, 0.0, 0.0, 0.0);
    auto lsm6d032Data = SensorLog::CreateLSM6D032Data(builder_, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    auto mplAltimeterData = SensorLog::CreateMPLAltimeterData(
        builder_,
        latestMPLAltimeter_.pressure,
        latestMPLAltimeter_.altitude);
    auto bno055Data = SensorLog::CreateBNO055Data(
        builder_,
        0.0, 0.0, 0.0, // accel_x, accel_y, accel_z
        0.0, 0.0, 0.0, // mag_x, mag_y, mag_z
        0.0, 0.0, 0.0, // gyro_x, gyro_y, gyro_z
        0.0, 0.0, 0.0, // euler_heading, euler_roll, euler_pitch
        0.0, 0.0, 0.0, // linear_accel_x, linear_accel_y, linear_accel_z
        0.0, 0.0, 0.0, // gravity_x, gravity_y, gravity_z
        0, 0, 0, 0     // calibration_status_system, gyro, accel, mag
    );

    // Serialize sensor message with timestamp and MPLAltimeter data
    auto sensorMessage = SensorLog::CreateSensorMessage(
        builder_,
        SensorLog::SensorType::SensorType_MPLAltimeter, // Example, adjust based on the current sensor
        timestamp,
        mplAltimeterData.Union()); // Use the correct union for the data

    builder_.Finish(sensorMessage);

    uint8_t *buf = builder_.GetBufferPointer();
    size_t size = builder_.GetSize();

    // Log the serialized message to SD card
    sdLogger_.logMessage(buf, size);

    // Reset builder for the next message
    builder_.Reset();

    // Periodically flush the buffer to ensure data is written
    if (timestamp % CALC_INTERVAL_MS < 10)
    { // Adjust threshold as needed
        sdLogger_.flush();
    }
}

void SensorManager::printAllData() const
{
    Serial.print("Timestamp: ");
    Serial.println(millis());

    Serial.print("BME688 - Temperature: ");
    Serial.print(latestBME688_.temperature);
    Serial.println(" °C");

    Serial.print("BME688 - Pressure: ");
    Serial.print(latestBME688_.pressure);
    Serial.println(" hPa");

    Serial.print("BME688 - Humidity: ");
    Serial.print(latestBME688_.humidity);
    Serial.println(" %");

    Serial.print("BME688 - Gas Resistance: ");
    Serial.print(latestBME688_.gas_resistance);
    Serial.println(" KOhms");

    Serial.print("BME688 - Altitude: ");
    Serial.print(latestBME688_.altitude);
    Serial.println(" m");

    // Repeat for other sensors...
    // ENS160, LSM6D032, MPLAltimeter, BNO055
}

void SensorManager::addSensor(std::shared_ptr<Sensor> sensor)
{
    sensors.push_back(sensor);
}

float SensorManager::getUpdateRateHz() const
{
    return updateRateHz_;
}
