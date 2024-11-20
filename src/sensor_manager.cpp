// sensor_manager
#include "sensor_manager.h"
#include "mpl_altimeter.h"
#include "sensor_struct.h"

SensorManager::SensorManager(uint8_t sdChipSelectPin)
    : lastUpdateTime_(0), updateCount_(0), updateRateHz_(0.0f),
      sdLogger_(sdChipSelectPin), builder_(2048), // Initialize builder with 1KB buffer
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

void SensorManager::addSensor(std::shared_ptr<Sensor> sensor)
{
    sensors.push_back(sensor);
}

void SensorManager::logAllData()
{
    unsigned long timestamp = millis();

    // Vector to hold all SensorMessage offsets
    std::vector<flatbuffers::Offset<SensorLog::SensorMessage>> messageVector;

    // Iterate through each sensor and serialize its data if new data is available
    for (auto &sensor : sensors)
    {
        if (sensor->hasNewData())
        {
            // Serialize the sensor's data and add to messageVector
            auto sensorMessage = sensor->serialize(builder_, timestamp);
            messageVector.push_back(sensorMessage);

            // Reset the new data flag
            sensor->resetNewDataFlag();
        }
    }

    // After collecting all SensorMessages, create SensorBatch
    if (!messageVector.empty())
    {
        // Create a FlatBuffers vector from the messageVector
        auto messages = builder_.CreateVector(messageVector);

        // Create the SensorBatch with the common timestamp and all messages
        auto sensorBatch = SensorLog::CreateSensorBatch(
            builder_,
            timestamp, // Common timestamp for the batch
            messages   // Vector of SensorMessage
        );

        // Finish the FlatBuffer with SensorBatch as the root
        builder_.Finish(sensorBatch);

        uint8_t *buf = builder_.GetBufferPointer();
        size_t size = builder_.GetSize();

        // Log the serialized batch to SD card
        sdLogger_.logMessage(buf, size);

        // Attemping deserializing
    
        Serial.println("Serialization...");

        // Optional: Check serialization (for debugging)
        deserializeAndVerify(buf, size);

        // Reset builder for the next message
        builder_.Reset();
    }

    // Periodically flush the buffer to ensure data is written
    if (timestamp % CALC_INTERVAL_MS < 10)
    { // Adjust threshold as needed
        sdLogger_.flush();
    }
}

void SensorManager::deserializeAndVerify(const uint8_t *buf, size_t size)
{
    // Verify the buffer
    flatbuffers::Verifier verifier(buf, size);
    if (!SensorLog::VerifySensorBatchBuffer(verifier))
    {
        Serial.println("Failed to verify SensorBatch buffer.");
        return;
    }

    // Get the root object
    auto sensorBatch = SensorLog::GetSensorBatch(buf);

    // Access and print the common timestamp
    auto batchTimestamp = sensorBatch->timestamp();
    Serial.print("Batch Timestamp: ");
    Serial.println(batchTimestamp);

    // Access the vector of SensorMessages
    auto messages = sensorBatch->messages();
    if (messages)
    {
        for (auto it = messages->begin(); it != messages->end(); ++it)
        {
            auto sensorMessage = *it;
            if (!sensorMessage)
                continue;

            // Access and print the sensor_type and timestamp for each message
            auto sensorType = sensorMessage->sensor_type();
            Serial.print("Sensor Type: ");
            switch (sensorType)
            {
            case SensorLog::SensorType_BME688:
                Serial.println("BME688");
                break;
            case SensorLog::SensorType_ENS160:
                Serial.println("ENS160");
                break;
            case SensorLog::SensorType_LSM6D032:
                Serial.println("LSM6D032");
                break;
            case SensorLog::SensorType_MPLAltimeter:
                Serial.println("MPLAltimeter");
                break;
            case SensorLog::SensorType_BNO055:
                Serial.println("BNO055");
                break;
            default:
                Serial.println("Unknown");
                break;
            }

            Serial.print("Message Timestamp: ");
            Serial.println(sensorMessage->timestamp());

            // Access the union data based on the sensor_type
            switch (sensorType)
            {
            case SensorLog::SensorType_BME688:
            {
                auto data = sensorMessage->data_as_BME688Data();
                if (data)
                {
                    Serial.println("BME688 Data:");
                    Serial.print("Temperature: ");
                    Serial.println(data->temperature());
                    Serial.print("Pressure: ");
                    Serial.println(data->pressure());
                    Serial.print("Humidity: ");
                    Serial.println(data->humidity());
                    Serial.print("Gas Resistance: ");
                    Serial.println(data->gas_resistance());
                    Serial.print("Altitude: ");
                    Serial.println(data->altitude());
                }
                break;
            }
            case SensorLog::SensorType_ENS160:
            {
                auto data = sensorMessage->data_as_ENS160Data();
                if (data)
                {
                    Serial.println("ENS160 Data:");
                    Serial.print("AQI: ");
                    Serial.println(data->aqi());
                    Serial.print("TVOC: ");
                    Serial.println(data->tvoc());
                    Serial.print("eCO2: ");
                    Serial.println(data->eco2());
                    Serial.print("HP0: ");
                    Serial.println(data->hp0());
                    Serial.print("HP1: ");
                    Serial.println(data->hp1());
                    Serial.print("HP2: ");
                    Serial.println(data->hp2());
                    Serial.print("HP3: ");
                    Serial.println(data->hp3());
                }
                break;
            }
            case SensorLog::SensorType_LSM6D032:
            {
                auto data = sensorMessage->data_as_LSM6D032Data();
                if (data)
                {
                    Serial.println("LSM6D032 Data:");
                    Serial.print("Accel X: ");
                    Serial.println(data->accel_x());
                    Serial.print("Accel Y: ");
                    Serial.println(data->accel_y());
                    Serial.print("Accel Z: ");
                    Serial.println(data->accel_z());
                    Serial.print("Gyro X: ");
                    Serial.println(data->gyro_x());
                    Serial.print("Gyro Y: ");
                    Serial.println(data->gyro_y());
                    Serial.print("Gyro Z: ");
                    Serial.println(data->gyro_z());
                }
                break;
            }
            case SensorLog::SensorType_MPLAltimeter:
            {
                auto data = sensorMessage->data_as_MPLAltimeterData();
                if (data)
                {
                    Serial.println("MPLAltimeter Data:");
                    Serial.print("Pressure: ");
                    Serial.println(data->pressure());
                    Serial.print("Altitude: ");
                    Serial.println(data->altitude());
                }
                break;
            }
            case SensorLog::SensorType_BNO055:
            {
                auto data = sensorMessage->data_as_BNO055Data();
                if (data)
                {
                    Serial.println("BNO055 Data:");
                    Serial.print("Accel X: ");
                    Serial.println(data->accel_x());
                    Serial.print("Accel Y: ");
                    Serial.println(data->accel_y());
                    Serial.print("Accel Z: ");
                    Serial.println(data->accel_z());
                    Serial.print("Mag X: ");
                    Serial.println(data->mag_x());
                    Serial.print("Mag Y: ");
                    Serial.println(data->mag_y());
                    Serial.print("Mag Z: ");
                    Serial.println(data->mag_z());
                    Serial.print("Gyro X: ");
                    Serial.println(data->gyro_x());
                    Serial.print("Gyro Y: ");
                    Serial.println(data->gyro_y());
                    Serial.print("Gyro Z: ");
                    Serial.println(data->gyro_z());
                    Serial.print("Euler Heading: ");
                    Serial.println(data->euler_heading());
                    Serial.print("Euler Roll: ");
                    Serial.println(data->euler_roll());
                    Serial.print("Euler Pitch: ");
                    Serial.println(data->euler_pitch());
                    Serial.print("Linear Accel X: ");
                    Serial.println(data->linear_accel_x());
                    Serial.print("Linear Accel Y: ");
                    Serial.println(data->linear_accel_y());
                    Serial.print("Linear Accel Z: ");
                    Serial.println(data->linear_accel_z());
                    Serial.print("Gravity X: ");
                    Serial.println(data->gravity_x());
                    Serial.print("Gravity Y: ");
                    Serial.println(data->gravity_y());
                    Serial.print("Gravity Z: ");
                    Serial.println(data->gravity_z());
                    Serial.print("Calibration Status System: ");
                    Serial.println(data->calibration_status_system());
                    Serial.print("Calibration Status Gyro: ");
                    Serial.println(data->calibration_status_gyro());
                    Serial.print("Calibration Status Accel: ");
                    Serial.println(data->calibration_status_accel());
                    Serial.print("Calibration Status Mag: ");
                    Serial.println(data->calibration_status_mag());
                }
                break;
            }
            default:
                Serial.println("Unknown Sensor Type. Cannot deserialize data.");
                break;
            }
        }
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

float SensorManager::getUpdateRateHz() const
{
    return updateRateHz_;
}
