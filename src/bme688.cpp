// bme688.cpp
#include "bme688.h"
#include "sensor_struct.h"

BME688Sensor::BME688Sensor() : sensor(&Wire) {}

bool BME688Sensor::begin()
{
    // Try to initialize!
    if (!sensor.begin())
    {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");

        return false;
    }

    sensor.setTemperatureOversampling(BME680_OS_8X);
    sensor.setHumidityOversampling(BME680_OS_2X);
    sensor.setPressureOversampling(BME680_OS_4X);
    sensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
    sensor.setGasHeater(320, 150); // 320*C for 150 ms

    return true;

}

void BME688Sensor::update()
{
    // Check if a new measurement is available
    if (!sensor.performReading())
    {
        Serial.println("Failed to perform reading :(");
        return;
    }

    float temperature = sensor.temperature;                     // in Celsius
    float pressure = sensor.pressure / 100.0;                   // in hPa
    float humidity = sensor.humidity;                           // in %
    float gas = sensor.gas_resistance / 1000.0;                 // in KOhms
    float altitude = sensor.readAltitude(SEALEVELPRESSURE_HPA); // in meters

    data_.temperature = sensor.temperature;
    data_.pressure = sensor.pressure;
    data_.humidity = sensor.humidity;
    data_.gas_resistance = sensor.gas_resistance;
    data_.altitude = sensor.readAltitude(SEALEVELPRESSURE_HPA);

    newDataFlag_ = true;

}

flatbuffers::Offset<SensorLog::SensorMessage> BME688Sensor::serialize(flatbuffers::FlatBufferBuilder &builder, unsigned long timestamp) const
{
    // Create FlatBuffers MPLAltimeterData from the struct
    auto bme = SensorLog::CreateBME688Data(builder, 0, 0, 0, 0, 0);

    // Get the union offset
    auto dataOffset = bme.Union();

    // Create the SensorMessage FlatBuffers object
    return SensorLog::CreateSensorMessage(
        builder,
        SensorLog::SensorType_BME688,          // sensor_type
        timestamp,                               // timestamp
        SensorLog::SensorDataUnion_BME688Data, // data_type (union type)
        dataOffset                               // data (union data)
    );
}

bool BME688Sensor::hasNewData() const
{
    return newDataFlag_;
}

SensorType BME688Sensor::getSensorType() const
{
    return SensorType::BME688;
}

void BME688Sensor::resetNewDataFlag()
{
    newDataFlag_ = false;
}

String BME688Sensor::getName() const
{
    return "BME688";
}

const SensorData *BME688Sensor::getData() const
{
    return &data_;
}

unsigned long BME688Sensor::getUpdateInterval() const
{
    return 1; // Approximately 500 Hz (1000 ms / 500 ≈ 2 ms)
}
