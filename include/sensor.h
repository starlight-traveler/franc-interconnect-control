#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "flatbuffers/flatbuffers.h"
#include "sdlogger.h"
#include "sensors_generated.h"
#include "sensor_struct.h"

class Sensor
{
public:
    virtual bool begin() = 0;                            // Initialize the sensor
    virtual void update() = 0;                           // Update sensor data
    virtual String getName() const = 0;                  // Get sensor name
    virtual String getData() const = 0;                  // Get sensor data as string
    virtual unsigned long getUpdateInterval() const = 0; // Get desired update interval in ms
    virtual bool hasNewData() const = 0;                 // Check if new data is available
    virtual void resetNewDataFlag() = 0;                 // Reset the new data flag
    virtual flatbuffers::Offset<SensorLog::SensorMessage> serialize(flatbuffers::FlatBufferBuilder &builder, ulong timestamp) const = 0;
    virtual ~Sensor() {}
};

#endif // SENSOR_H
