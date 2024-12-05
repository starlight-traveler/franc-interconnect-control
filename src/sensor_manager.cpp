#include "sensor_manager.h"
#include "mpl_altimeter.h"
#include "sensor_struct.h"

SensorManager::SensorManager(uint8_t sdChipSelectPin)
    : lastUpdateTime_(0), updateCount_(0), updateRateHz_(0.0f),
      csvLogger_(sdChipSelectPin, "log.csv"), builder_(2048), // Initialize builder with 2KB buffer
      latestBME688_(), latestENS160_(), latestLSM6D032_(),
      latestMPLAltimeter_(), latestBNO055_(),
      headersWritten_(false) // Initialize the flag for header writing
{
}

bool SensorManager::begin()
{
    // Initialize CSV Logger
    if (!csvLogger_.begin())
    {
        Serial.println("Failed to initialize CSV Logger.");
        return false;
    }
    return true;
}

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

        // Serial.println(updateRateHz_);
    }
}

void SensorManager::addSensor(std::shared_ptr<Sensor> sensor)
{
    sensors.push_back(sensor);
}

void SensorManager::logAllData()
{
    unsigned long timestamp = millis();

    // Log data to CSV
    logToCSV(timestamp);

    // Periodically flush the buffers to ensure data is written
    if (timestamp % CALC_INTERVAL_MS < 10)
    {
        csvLogger_.flush();
    }
}

void SensorManager::logToCSV(unsigned long timestamp)
{
    // Write headers if not already written
    if (!headersWritten_)
    {
        String headers = "Timestamp,Sensor,BME_Temperature,BME_Pressure,BME_Humidity,BME_Gas_Resistance,BME_Altitude,ENS_AQI,ENS_TVOC,ENS_eCO2,ENS_HP0,ENS_HP1,ENS_HP2,ENS_HP3,LSM_Accel_X,LSM_Accel_Y,LSM_Accel_Z,LSM_Gyro_X,LSM_Gyro_Y,LSM_Gyro_Z,MPL_Pressure,MPL_Altitude,BNO_Accel_X,BNO_Accel_Y,BNO_Accel_Z,BNO_Mag_X,BNO_Mag_Y,BNO_Mag_Z,BNO_Gyro_X,BNO_Gyro_Y,BNO_Gyro_Z,BNO_Euler_Heading,BNO_Euler_Roll,BNO_Euler_Pitch,BNO_Linear_Accel_X,BNO_Linear_Accel_Y,BNO_Linear_Accel_Z,BNO_Gravity_X,BNO_Gravity_Y,BNO_Gravity_Z,BNO_Calibration_System,BNO_Calibration_Gyro,BNO_Calibration_Accel,BNO_Calibration_Mag";
        csvLogger_.logCSV(headers.c_str());
        headersWritten_ = true;
    }

    // Initialize all fields to empty strings or default values
    String sensorName = "";
    String bmeTemperature = "";
    String bmePressure = "";
    String bmeHumidity = "";
    String bmeGasResistance = "";
    String bmeAltitude = "";
    String ensAQI = "";
    String ensTVOC = "";
    String enseCO2 = "";
    String ensHP0 = "";
    String ensHP1 = "";
    String ensHP2 = "";
    String ensHP3 = "";
    String lsmAccelX = "";
    String lsmAccelY = "";
    String lsmAccelZ = "";
    String lsmGyroX = "";
    String lsmGyroY = "";
    String lsmGyroZ = "";
    String mplPressure = "";
    String mplAltitude = "";
    String bnoAccelX = "";
    String bnoAccelY = "";
    String bnoAccelZ = "";
    String bnoMagX = "";
    String bnoMagY = "";
    String bnoMagZ = "";
    String bnoGyroX = "";
    String bnoGyroY = "";
    String bnoGyroZ = "";
    String bnoEulerHeading = "";
    String bnoEulerRoll = "";
    String bnoEulerPitch = "";
    String bnoLinearAccelX = "";
    String bnoLinearAccelY = "";
    String bnoLinearAccelZ = "";
    String bnoGravityX = "";
    String bnoGravityY = "";
    String bnoGravityZ = "";
    String bnoCalibrationSystem = "";
    String bnoCalibrationGyro = "";
    String bnoCalibrationAccel = "";
    String bnoCalibrationMag = "";

    // Collect data from sensors
    for (auto &sensor : sensors)
    {
        if (sensor->hasNewData())
        {
            sensorName = sensor->getName();
            const SensorData *data = sensor->getData();
            SensorType type = sensor->getSensorType();

            switch (type)
            {
            case SensorType::BME688:
            {
                const BME688DataStruct *bmeData = static_cast<const BME688DataStruct *>(data);
                bmeTemperature = String(bmeData->temperature, 2);
                bmePressure = String(bmeData->pressure, 2);
                bmeHumidity = String(bmeData->humidity, 2);
                bmeGasResistance = String(bmeData->gas_resistance, 2);
                bmeAltitude = String(bmeData->altitude, 2);
                break;
            }
            case SensorType::ENS160:
            {
                const ENS160DataStruct *ensData = static_cast<const ENS160DataStruct *>(data);
                ensAQI = String(ensData->aqi);
                ensTVOC = String(ensData->tvoc);
                enseCO2 = String(ensData->eco2);
                ensHP0 = String(ensData->hp0, 2);
                ensHP1 = String(ensData->hp1, 2);
                ensHP2 = String(ensData->hp2, 2);
                ensHP3 = String(ensData->hp3, 2);
                break;
            }
            case SensorType::LSM6D032:
            {
                const LSM6D032DataStruct *lsmData = static_cast<const LSM6D032DataStruct *>(data);
                lsmAccelX = String(lsmData->accel_x, 2);
                lsmAccelY = String(lsmData->accel_y, 2);
                lsmAccelZ = String(lsmData->accel_z, 2);
                lsmGyroX = String(lsmData->gyro_x, 2);
                lsmGyroY = String(lsmData->gyro_y, 2);
                lsmGyroZ = String(lsmData->gyro_z, 2);
                break;
            }
            case SensorType::MPLAltimeter:
            {
                const MPLAltimeterDataStruct *mplData = static_cast<const MPLAltimeterDataStruct *>(data);
                mplPressure = String(mplData->pressure, 2);
                mplAltitude = String(mplData->altitude, 2);
                break;
            }
            case SensorType::BNO055:
            {
                const BNO055DataStruct *bnoData = static_cast<const BNO055DataStruct *>(data);
                bnoAccelX = String(bnoData->accel_x, 2);
                bnoAccelY = String(bnoData->accel_y, 2);
                bnoAccelZ = String(bnoData->accel_z, 2);
                bnoMagX = String(bnoData->mag_x, 2);
                bnoMagY = String(bnoData->mag_y, 2);
                bnoMagZ = String(bnoData->mag_z, 2);
                bnoGyroX = String(bnoData->gyro_x, 2);
                bnoGyroY = String(bnoData->gyro_y, 2);
                bnoGyroZ = String(bnoData->gyro_z, 2);
                bnoEulerHeading = String(bnoData->euler_heading, 2);
                bnoEulerRoll = String(bnoData->euler_roll, 2);
                bnoEulerPitch = String(bnoData->euler_pitch, 2);
                bnoLinearAccelX = String(bnoData->linear_accel_x, 2);
                bnoLinearAccelY = String(bnoData->linear_accel_y, 2);
                bnoLinearAccelZ = String(bnoData->linear_accel_z, 2);
                bnoGravityX = String(bnoData->gravity_x, 2);
                bnoGravityY = String(bnoData->gravity_y, 2);
                bnoGravityZ = String(bnoData->gravity_z, 2);
                bnoCalibrationSystem = String(bnoData->calibration_status_system);
                bnoCalibrationGyro = String(bnoData->calibration_status_gyro);
                bnoCalibrationAccel = String(bnoData->calibration_status_accel);
                bnoCalibrationMag = String(bnoData->calibration_status_mag);
                break;
            }
            default:
                Serial.println("Unknown sensor type encountered.");
                break;
            }

            // Reset the new data flag
            sensor->resetNewDataFlag();
        }
    }

    // Build the CSV line with all fields
    String csvLine = String(timestamp) + "," + "FRANC" + "," +
                     bmeTemperature + "," + bmePressure + "," + bmeHumidity + "," + bmeGasResistance + "," + bmeAltitude + "," +
                     ensAQI + "," + ensTVOC + "," + enseCO2 + "," + ensHP0 + "," + ensHP1 + "," + ensHP2 + "," + ensHP3 + "," +
                     lsmAccelX + "," + lsmAccelY + "," + lsmAccelZ + "," + lsmGyroX + "," + lsmGyroY + "," + lsmGyroZ + "," +
                     mplPressure + "," + mplAltitude + "," +
                     bnoAccelX + "," + bnoAccelY + "," + bnoAccelZ + "," +
                     bnoMagX + "," + bnoMagY + "," + bnoMagZ + "," +
                     bnoGyroX + "," + bnoGyroY + "," + bnoGyroZ + "," +
                     bnoEulerHeading + "," + bnoEulerRoll + "," + bnoEulerPitch + "," +
                     bnoLinearAccelX + "," + bnoLinearAccelY + "," + bnoLinearAccelZ + "," +
                     bnoGravityX + "," + bnoGravityY + "," + bnoGravityZ + "," +
                     bnoCalibrationSystem + "," + bnoCalibrationGyro + "," + bnoCalibrationAccel + "," + bnoCalibrationMag +
                     "\n";

    // Write the CSV line to the CSV logger
    csvLogger_.logCSV(csvLine.c_str());
}

float SensorManager::getUpdateRateHz() const
{
    return updateRateHz_;
}
