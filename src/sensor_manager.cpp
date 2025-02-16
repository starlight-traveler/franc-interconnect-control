#include "sensor_manager.h"
#include "mpl_altimeter.h"
#include "sensor_struct.h"

#define SENSOR_UPDATE_TIMEOUT_MS 50

// A helper function that wraps vsnprintf() and, in case of error,
// replaces the buffer with an error message.
static void safe_snprintf(char *buffer, size_t size, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int ret = vsnprintf(buffer, size, format, args);
    va_end(args);
    if (ret < 0 || (size_t)ret >= size)
    {
        // Formatting error or truncation; replace with "ERR"
        strncpy(buffer, "ERR", size);
        buffer[size - 1] = '\0';
    }
}

SensorManager::SensorManager(uint8_t sdChipSelectPin)
    : lastUpdateTime_(0),
      updateCount_(0),
      updateRateHz_(0.0f),
      csvLogger_(sdChipSelectPin, "logger.csv"),
      headersWritten_(false)
{
}

bool SensorManager::begin()
{
    // Try to initialize the CSV Logger up to 3 times before giving up.
    const int maxRetries = 3;
    int attempt = 0;
    while (attempt < maxRetries && !csvLogger_.begin())
    {
        Serial.print("CSV Logger initialization failed. Retry ");
        Serial.println(attempt + 1);
        delay(500);
        attempt++;
    }
    if (attempt == maxRetries)
    {
        Serial.println("CSV Logger failed to initialize after retries.");
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
            Serial.println(" failed to initialize. Attempting reinitialization.");
            delay(100); // small delay before retrying
            if (!sensor->begin())
            {
                Serial.print(sensor->getName());
                Serial.println(" reinitialization failed.");
                allInitialized = false;
            }
            else
            {
                Serial.print(sensor->getName());
                Serial.println(" reinitialized successfully.");
            }
        }
    }
    return allInitialized;
}

void SensorManager::updateAll()
{
    unsigned long startTime = millis();
    for (auto &sensor : sensors)
    {
        sensor->update();
        // If sensor update is taking too long, warn and skip remaining sensors.
        if (millis() - startTime > SENSOR_UPDATE_TIMEOUT_MS)
        {
            Serial.println("Warning: Sensor update took too long; skipping remaining sensors.");
            break;
        }
    }

    // Update cycle count and compute update rate periodically.
    updateCount_++;
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime_ >= CALC_INTERVAL_MS)
    {
        updateRateHz_ = (float)updateCount_ / ((currentTime - lastUpdateTime_) / 1000.0f);
        lastUpdateTime_ = currentTime;
        updateCount_ = 0;
    }
}

void SensorManager::addSensor(std::shared_ptr<Sensor> sensor)
{
    sensors.push_back(sensor);
}

void SensorManager::logAllData()
{
    unsigned long timestamp = millis();
    logToCSV(timestamp);

    // Flush periodically to ensure data is written.
    if (timestamp % CALC_INTERVAL_MS < 10)
    {
        csvLogger_.flush();
    }
}

void SensorManager::logToCSV(unsigned long timestamp)
{
    // Write the CSV header only once.
    if (!headersWritten_)
    {
        const char header[] =
            "Timestamp,Sensor,BME_Temperature,BME_Pressure,BME_Humidity,BME_Gas_Resistance,BME_Altitude,"
            "ENS_AQI,ENS_TVOC,ENS_eCO2,ENS_HP0,ENS_HP1,ENS_HP2,ENS_HP3,"
            "LSM_Accel_X,LSM_Accel_Y,LSM_Accel_Z,LSM_Gyro_X,LSM_Gyro_Y,LSM_Gyro_Z,"
            "MPL_Pressure,MPL_Altitude,"
            "BNO_Accel_X,BNO_Accel_Y,BNO_Accel_Z,"
            "BNO_Mag_X,BNO_Mag_Y,BNO_Mag_Z,"
            "BNO_Gyro_X,BNO_Gyro_Y,BNO_Gyro_Z,"
            "BNO_Euler_Heading,BNO_Euler_Roll,BNO_Euler_Pitch,"
            "BNO_Linear_Accel_X,BNO_Linear_Accel_Y,BNO_Linear_Accel_Z,"
            "BNO_Gravity_X,BNO_Gravity_Y,BNO_Gravity_Z,"
            "BNO_Calibration_System,BNO_Calibration_Gyro,BNO_Calibration_Accel,BNO_Calibration_Mag";
        csvLogger_.logCSV(header);
        headersWritten_ = true;
    }

    // Use fixed-size buffers for each sensor field to reduce dynamic allocation.
    char bmeTemperature[16] = "";
    char bmePressure[16] = "";
    char bmeHumidity[16] = "";
    char bmeGasResistance[16] = "";
    char bmeAltitude[16] = "";

    char ensAQI[16] = "";
    char ensTVOC[16] = "";
    char enseCO2[16] = "";
    char ensHP0[16] = "";
    char ensHP1[16] = "";
    char ensHP2[16] = "";
    char ensHP3[16] = "";

    char lsmAccelX[16] = "";
    char lsmAccelY[16] = "";
    char lsmAccelZ[16] = "";
    char lsmGyroX[16] = "";
    char lsmGyroY[16] = "";
    char lsmGyroZ[16] = "";

    char mplPressure[16] = "";
    char mplAltitude[16] = "";

    char bnoAccelX[16] = "";
    char bnoAccelY[16] = "";
    char bnoAccelZ[16] = "";

    char bnoMagX[16] = "";
    char bnoMagY[16] = "";
    char bnoMagZ[16] = "";

    char bnoGyroX[16] = "";
    char bnoGyroY[16] = "";
    char bnoGyroZ[16] = "";

    char bnoEulerHeading[16] = "";
    char bnoEulerRoll[16] = "";
    char bnoEulerPitch[16] = "";

    char bnoLinearAccelX[16] = "";
    char bnoLinearAccelY[16] = "";
    char bnoLinearAccelZ[16] = "";

    char bnoGravityX[16] = "";
    char bnoGravityY[16] = "";
    char bnoGravityZ[16] = "";

    char bnoCalibrationSystem[16] = "";
    char bnoCalibrationGyro[16] = "";
    char bnoCalibrationAccel[16] = "";
    char bnoCalibrationMag[16] = "";

    // Loop through each sensor that has new data.
    for (auto &sensor : sensors)
    {
        if (sensor->hasNewData())
        {
            const SensorData *data = sensor->getData();
            if (data == nullptr)
            {
                Serial.print("Error: No data received from sensor ");
                Serial.println(sensor->getName());
                continue;
            }
            SensorType type = sensor->getSensorType();
            switch (type)
            {
            case SensorType::BME688:
            {
                const BME688DataStruct *bmeData = static_cast<const BME688DataStruct *>(data);
                safe_snprintf(bmeTemperature, sizeof(bmeTemperature), "%.2f", bmeData->temperature);
                safe_snprintf(bmePressure, sizeof(bmePressure), "%.2f", bmeData->pressure);
                safe_snprintf(bmeHumidity, sizeof(bmeHumidity), "%.2f", bmeData->humidity);
                safe_snprintf(bmeGasResistance, sizeof(bmeGasResistance), "%.2f", bmeData->gas_resistance);
                safe_snprintf(bmeAltitude, sizeof(bmeAltitude), "%.2f", bmeData->altitude);
                break;
            }
            case SensorType::ENS160:
            {
                const ENS160DataStruct *ensData = static_cast<const ENS160DataStruct *>(data);
                safe_snprintf(ensAQI, sizeof(ensAQI), "%d", ensData->aqi);
                safe_snprintf(ensTVOC, sizeof(ensTVOC), "%d", ensData->tvoc);
                safe_snprintf(enseCO2, sizeof(enseCO2), "%d", ensData->eco2);
                safe_snprintf(ensHP0, sizeof(ensHP0), "%.2f", ensData->hp0);
                safe_snprintf(ensHP1, sizeof(ensHP1), "%.2f", ensData->hp1);
                safe_snprintf(ensHP2, sizeof(ensHP2), "%.2f", ensData->hp2);
                safe_snprintf(ensHP3, sizeof(ensHP3), "%.2f", ensData->hp3);
                break;
            }
            case SensorType::LSM6D032:
            {
                const LSM6D032DataStruct *lsmData = static_cast<const LSM6D032DataStruct *>(data);
                safe_snprintf(lsmAccelX, sizeof(lsmAccelX), "%.2f", lsmData->accel_x);
                safe_snprintf(lsmAccelY, sizeof(lsmAccelY), "%.2f", lsmData->accel_y);
                safe_snprintf(lsmAccelZ, sizeof(lsmAccelZ), "%.2f", lsmData->accel_z);
                safe_snprintf(lsmGyroX, sizeof(lsmGyroX), "%.2f", lsmData->gyro_x);
                safe_snprintf(lsmGyroY, sizeof(lsmGyroY), "%.2f", lsmData->gyro_y);
                safe_snprintf(lsmGyroZ, sizeof(lsmGyroZ), "%.2f", lsmData->gyro_z);
                break;
            }
            case SensorType::MPLAltimeter:
            {
                const MPLAltimeterDataStruct *mplData = static_cast<const MPLAltimeterDataStruct *>(data);
                safe_snprintf(mplPressure, sizeof(mplPressure), "%.2f", mplData->pressure);
                safe_snprintf(mplAltitude, sizeof(mplAltitude), "%.2f", mplData->altitude);
                break;
            }
            case SensorType::BNO055:
            {
                const BNO055DataStruct *bnoData = static_cast<const BNO055DataStruct *>(data);
                safe_snprintf(bnoAccelX, sizeof(bnoAccelX), "%.2f", bnoData->accel_x);
                safe_snprintf(bnoAccelY, sizeof(bnoAccelY), "%.2f", bnoData->accel_y);
                safe_snprintf(bnoAccelZ, sizeof(bnoAccelZ), "%.2f", bnoData->accel_z);
                safe_snprintf(bnoMagX, sizeof(bnoMagX), "%.2f", bnoData->mag_x);
                safe_snprintf(bnoMagY, sizeof(bnoMagY), "%.2f", bnoData->mag_y);
                safe_snprintf(bnoMagZ, sizeof(bnoMagZ), "%.2f", bnoData->mag_z);
                safe_snprintf(bnoGyroX, sizeof(bnoGyroX), "%.2f", bnoData->gyro_x);
                safe_snprintf(bnoGyroY, sizeof(bnoGyroY), "%.2f", bnoData->gyro_y);
                safe_snprintf(bnoGyroZ, sizeof(bnoGyroZ), "%.2f", bnoData->gyro_z);
                safe_snprintf(bnoEulerHeading, sizeof(bnoEulerHeading), "%.2f", bnoData->euler_heading);
                safe_snprintf(bnoEulerRoll, sizeof(bnoEulerRoll), "%.2f", bnoData->euler_roll);
                safe_snprintf(bnoEulerPitch, sizeof(bnoEulerPitch), "%.2f", bnoData->euler_pitch);
                safe_snprintf(bnoLinearAccelX, sizeof(bnoLinearAccelX), "%.2f", bnoData->linear_accel_x);
                safe_snprintf(bnoLinearAccelY, sizeof(bnoLinearAccelY), "%.2f", bnoData->linear_accel_y);
                safe_snprintf(bnoLinearAccelZ, sizeof(bnoLinearAccelZ), "%.2f", bnoData->linear_accel_z);
                safe_snprintf(bnoGravityX, sizeof(bnoGravityX), "%.2f", bnoData->gravity_x);
                safe_snprintf(bnoGravityY, sizeof(bnoGravityY), "%.2f", bnoData->gravity_y);
                safe_snprintf(bnoGravityZ, sizeof(bnoGravityZ), "%.2f", bnoData->gravity_z);
                safe_snprintf(bnoCalibrationSystem, sizeof(bnoCalibrationSystem), "%d", bnoData->calibration_status_system);
                safe_snprintf(bnoCalibrationGyro, sizeof(bnoCalibrationGyro), "%d", bnoData->calibration_status_gyro);
                safe_snprintf(bnoCalibrationAccel, sizeof(bnoCalibrationAccel), "%d", bnoData->calibration_status_accel);
                safe_snprintf(bnoCalibrationMag, sizeof(bnoCalibrationMag), "%d", bnoData->calibration_status_mag);
                break;
            }
            default:
                Serial.println("Unknown sensor type encountered.");
                break;
            }
            sensor->resetNewDataFlag();
        }
    }

    // Build the complete CSV line using a fixed-size buffer.
    char csvLine[1024];
    int written = snprintf(csvLine, sizeof(csvLine),
                           "%lu,FRANC,%s,%s,%s,%s,%s," // Timestamp, Sensor, BME688 fields
                           "%s,%s,%s,%s,%s,%s,%s,"     // ENS160 fields
                           "%s,%s,%s,%s,%s,%s,"        // LSM6D032 fields
                           "%s,%s,"                    // MPLAltimeter fields
                           "%s,%s,%s,"                 // BNO055 Accel fields
                           "%s,%s,%s,"                 // BNO055 Mag fields
                           "%s,%s,%s,"                 // BNO055 Gyro fields
                           "%s,%s,%s,"                 // BNO055 Euler fields
                           "%s,%s,%s,"                 // BNO055 Linear Accel fields
                           "%s,%s,%s,"                 // BNO055 Gravity fields
                           "%s,%s,%s,%s\n",            // BNO055 Calibration fields
                           timestamp,
                           bmeTemperature, bmePressure, bmeHumidity, bmeGasResistance, bmeAltitude,
                           ensAQI, ensTVOC, enseCO2, ensHP0, ensHP1, ensHP2, ensHP3,
                           lsmAccelX, lsmAccelY, lsmAccelZ, lsmGyroX, lsmGyroY, lsmGyroZ,
                           mplPressure, mplAltitude,
                           bnoAccelX, bnoAccelY, bnoAccelZ,
                           bnoMagX, bnoMagY, bnoMagZ,
                           bnoGyroX, bnoGyroY, bnoGyroZ,
                           bnoEulerHeading, bnoEulerRoll, bnoEulerPitch,
                           bnoLinearAccelX, bnoLinearAccelY, bnoLinearAccelZ,
                           bnoGravityX, bnoGravityY, bnoGravityZ,
                           bnoCalibrationSystem, bnoCalibrationGyro, bnoCalibrationAccel, bnoCalibrationMag);
    if (written < 0 || written >= (int)sizeof(csvLine))
    {
        Serial.println("Error: CSV line truncated or formatting error.");
    }

    // Log the CSV line to the SD card.
    csvLogger_.logCSV(csvLine);
}

MasterSensorData SensorManager::getAggregatedData()
{
    MasterSensorData masterData = {};
    masterData.timestamp = millis();

    // Loop over each sensor that has new data.
    for (auto &sensor : sensors)
    {
        if (!sensor->hasNewData())
            continue;

        const SensorData *data = sensor->getData();
        if (data == nullptr)
            continue;

        switch (sensor->getSensorType())
        {
        case SensorType::BME688:
        {
            const BME688DataStruct *bmeData = static_cast<const BME688DataStruct *>(data);
            masterData.bme_temperature = bmeData->temperature;
            masterData.bme_pressure = bmeData->pressure;
            masterData.bme_humidity = bmeData->humidity;
            masterData.bme_gas_resistance = bmeData->gas_resistance;
            masterData.bme_altitude = bmeData->altitude;
            break;
        }
        case SensorType::ENS160:
        {
            const ENS160DataStruct *ensData = static_cast<const ENS160DataStruct *>(data);
            masterData.ens_aqi = ensData->aqi;
            masterData.ens_tvoc = ensData->tvoc;
            masterData.ens_eco2 = ensData->eco2;
            masterData.ens_hp0 = ensData->hp0;
            masterData.ens_hp1 = ensData->hp1;
            masterData.ens_hp2 = ensData->hp2;
            masterData.ens_hp3 = ensData->hp3;
            break;
        }
        case SensorType::LSM6D032:
        {
            const LSM6D032DataStruct *lsmData = static_cast<const LSM6D032DataStruct *>(data);
            masterData.lsm_accel_x = lsmData->accel_x;
            masterData.lsm_accel_y = lsmData->accel_y;
            masterData.lsm_accel_z = lsmData->accel_z;
            masterData.lsm_gyro_x = lsmData->gyro_x;
            masterData.lsm_gyro_y = lsmData->gyro_y;
            masterData.lsm_gyro_z = lsmData->gyro_z;
            break;
        }
        case SensorType::MPLAltimeter:
        {
            const MPLAltimeterDataStruct *mplData = static_cast<const MPLAltimeterDataStruct *>(data);
            masterData.mpl_pressure = mplData->pressure;
            masterData.mpl_altitude = mplData->altitude;
            break;
        }
        case SensorType::BNO055:
        {
            const BNO055DataStruct *bnoData = static_cast<const BNO055DataStruct *>(data);
            masterData.bno_accel_x = bnoData->accel_x;
            masterData.bno_accel_y = bnoData->accel_y;
            masterData.bno_accel_z = bnoData->accel_z;
            masterData.bno_mag_x = bnoData->mag_x;
            masterData.bno_mag_y = bnoData->mag_y;
            masterData.bno_mag_z = bnoData->mag_z;
            masterData.bno_gyro_x = bnoData->gyro_x;
            masterData.bno_gyro_y = bnoData->gyro_y;
            masterData.bno_gyro_z = bnoData->gyro_z;
            masterData.bno_euler_heading = bnoData->euler_heading;
            masterData.bno_euler_roll = bnoData->euler_roll;
            masterData.bno_euler_pitch = bnoData->euler_pitch;
            masterData.bno_linear_accel_x = bnoData->linear_accel_x;
            masterData.bno_linear_accel_y = bnoData->linear_accel_y;
            masterData.bno_linear_accel_z = bnoData->linear_accel_z;
            masterData.bno_gravity_x = bnoData->gravity_x;
            masterData.bno_gravity_y = bnoData->gravity_y;
            masterData.bno_gravity_z = bnoData->gravity_z;
            masterData.bno_calibration_system = bnoData->calibration_status_system;
            masterData.bno_calibration_gyro = bnoData->calibration_status_gyro;
            masterData.bno_calibration_accel = bnoData->calibration_status_accel;
            masterData.bno_calibration_mag = bnoData->calibration_status_mag;
            break;
        }
        default:
            // Unknown sensor type; do nothing.
            break;
        }
        sensor->resetNewDataFlag();
    }
    return masterData;
}

float SensorManager::getUpdateRateHz() const
{
    return updateRateHz_;
}
