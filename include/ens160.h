// ens160.h
#ifndef ENS160_SENSOR_H
#define ENS160_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include "ScioSense_ENS160.h" // ENS160 library

class ENS160Sensor : public Sensor
{
public:
    ENS160Sensor();
    bool begin() override;
    void update() override;
    String getName() const override;
    const SensorData *getData() const override;
    unsigned long getUpdateInterval() const override;

    bool hasNewData() const override;
    void resetNewDataFlag() override;
    virtual flatbuffers::Offset<SensorLog::SensorMessage> serialize(flatbuffers::FlatBufferBuilder &builder, unsigned long timestamp) const override;
    SensorType getSensorType() const override;

private:
    ScioSense_ENS160 ens160;
    ENS160DataStruct data_;
    bool newDataFlag_;
    unsigned long lastUpdateTime;

    enum MeasurementState
    {
        Idle,
        Measuring
    };
    MeasurementState measurementState_;
};

#endif // ENS160_SENSOR_H
