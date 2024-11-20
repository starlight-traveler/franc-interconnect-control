// MPLAltimeterSensor.h
#ifndef MPL_ALTIMETER_SENSOR_H
#define MPL_ALTIMETER_SENSOR_H

#include "sensor.h"
#include "sensor_struct.h"
#include "flatbuffers/flatbuffers.h"
#include "sensors_generated.h"

#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

class MPLAltimeterSensor : public Sensor
{
public:
    MPLAltimeterSensor();
    bool begin() override;
    void update() override;
    String getName() const override;
    String getData() const override;
    unsigned long getUpdateInterval() const override;

    bool hasNewData() const override;        // Indicates if there's new data available
    void resetNewDataFlag() override;        // Resets the new data flag
    void getData(void *data) const; // Retrieves the populated struct

    void setOversampleRate(uint8_t oversampleRate);
    virtual flatbuffers::Offset<SensorLog::SensorMessage> serialize(flatbuffers::FlatBufferBuilder &builder, unsigned long timestamp) const override;

private:
    Adafruit_MPL3115A2 mpl;       // MPL3115A2 sensor object
    MPLAltimeterDataStruct data_; // Struct to hold sensor data
    bool newDataFlag_;            // Tracks if new data is available
    unsigned long lastUpdateTime; // Tracks the last update time
    String sensor_data;
};

#endif // MPL_ALTIMETER_SENSOR_H