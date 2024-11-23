#ifndef LSM6D032_SENSOR_H
#define LSM6D032_SENSOR_H

#include "sensor.h"
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>

class LSM6D032Sensor : public Sensor
{
public:
    LSM6D032Sensor();
    bool begin() override;
    void update() override;
    void setReports();
    String getName() const override;
    const SensorData *getData() const override; // Updated return type
    unsigned long getUpdateInterval() const override;

    bool hasNewData() const override; // Indicates if there's new data available
    void resetNewDataFlag() override; // Resets the new data flag
    void getData(void *data) const;   // Retrieves the populated struct
    virtual flatbuffers::Offset<SensorLog::SensorMessage> serialize(flatbuffers::FlatBufferBuilder &builder, unsigned long timestamp) const override;
    SensorType getSensorType() const override; // Implemented method

private:
    Adafruit_LSM6DSO32 lsm;

    bool newDataFlag_;      // Tracks if new data is available
    LSM6D032DataStruct data_; // Struct to hold sensor data

    unsigned long lastUpdateTime;
    String sensorData_;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
};

#endif 
