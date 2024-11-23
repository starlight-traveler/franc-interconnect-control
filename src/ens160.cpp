// ens160.cpp
#include "ens160.h"

ENS160Sensor::ENS160Sensor()
    : ens160(ENS160_I2CADDR_1), newDataFlag_(false), lastUpdateTime(0), measurementState_(Idle) {}

bool ENS160Sensor::begin()
{
    if (!ens160.begin())
    {
        return false;
    }

    ens160.setMode(ENS160_OPMODE_STD);
    return true;
}

void ENS160Sensor::update()
{
    unsigned long currentTime = millis();
    unsigned long desiredInterval = getUpdateInterval();

    if (currentTime - lastUpdateTime >= desiredInterval)
    {
        lastUpdateTime = currentTime;

        switch (measurementState_)
        {
        case Idle:
            // Start a new measurement
            // For ENS160, measurements are continuous in standard mode, so no need to initiate
            measurementState_ = Measuring;
            break;

        case Measuring:
            if (ens160.available())
            {
                // Retrieve measurement data
                if (ens160.measure(true))
                {
                    if (ens160.measureRaw(true))
                    {
                        data_.aqi = ens160.getAQI();
                        data_.tvoc = ens160.getTVOC(); // in ppb
                        data_.eco2 = ens160.geteCO2(); // in ppm
                        data_.hp0 = ens160.getHP0();
                        data_.hp1 = ens160.getHP1();
                        data_.hp2 = ens160.getHP2();
                        data_.hp3 = ens160.getHP3();

                        newDataFlag_ = true;
                        measurementState_ = Idle; // Reset state
                    }
                    else
                    {
                        Serial.println("ENS160 raw measurement failed.");
                    }
                }
                else
                {
                    Serial.println("ENS160 standard measurement failed.");
                }
            }
            // If data not available, do nothing and return quickly
            break;
        }
    }
}

flatbuffers::Offset<SensorLog::SensorMessage> ENS160Sensor::serialize(flatbuffers::FlatBufferBuilder &builder, unsigned long timestamp) const
{
    auto ens160Data = SensorLog::CreateENS160Data(
        builder,
        data_.aqi,
        data_.tvoc,
        data_.eco2,
        data_.hp0,
        data_.hp1,
        data_.hp2,
        data_.hp3);

    auto dataOffset = ens160Data.Union();

    return SensorLog::CreateSensorMessage(
        builder,
        SensorLog::SensorType_ENS160,
        timestamp,
        SensorLog::SensorDataUnion_ENS160Data,
        dataOffset);
}

String ENS160Sensor::getName() const
{
    return "ENS160";
}

SensorType ENS160Sensor::getSensorType() const
{
    return SensorType::ENS160;
}

const SensorData *ENS160Sensor::getData() const
{
    return &data_;
}

void ENS160Sensor::resetNewDataFlag()
{
    newDataFlag_ = false;
}

bool ENS160Sensor::hasNewData() const
{
    return newDataFlag_;
}

unsigned long ENS160Sensor::getUpdateInterval() const
{
    return 1000; // 1 Hz update rate
}
