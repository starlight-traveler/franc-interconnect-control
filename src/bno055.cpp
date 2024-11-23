// bno085.cpp
#include "bno055.h"

BNO055Sensor::BNO055Sensor() : sensor(), newDataFlag_(false) {}

bool BNO055Sensor::begin()
{
    // Try to initialize!
    if (!sensor.begin())
    {
        return false;
    }

    setExternalCrystal();
    return true;
}

void BNO055Sensor::setExternalCrystal(void)
{
    sensor.setExtCrystalUse(true);
}

const SensorData *BNO055Sensor::getData() const
{
    return &data_;
}

SensorType BNO055Sensor::getSensorType() const
{
    return SensorType::BNO055;
}

void BNO055Sensor::update()
{

    // Define sensor event structures for each data type
    sensors_event_t accel_event, mag_event, gyro_event;
    sensors_event_t euler_event, quat_event, linear_accel_event, gravity_event;

    // Variables to hold calibration status
    uint8_t system, gyro_cal, accel_cal, mag_cal;

    // Retrieve Accelerometer data
    if (!sensor.getEvent(&accel_event, Adafruit_BNO055::VECTOR_ACCELEROMETER))
    {
        Serial.println("Failed to get accelerometer data");
        return;
    }

    // Acceleration
    data_.accel_x = accel_event.acceleration.x;
    data_.accel_y = accel_event.acceleration.y;
    data_.accel_z = accel_event.acceleration.z;

    // Retrieve Magnetometer data
    if (!sensor.getEvent(&mag_event, Adafruit_BNO055::VECTOR_MAGNETOMETER))
    {
        Serial.println("Failed to get magnetometer data");
        return;
    }

    // Magnometer
    data_.mag_x = mag_event.magnetic.x;
    data_.mag_y = mag_event.magnetic.y;
    data_.mag_z = mag_event.magnetic.z;

    // Retrieve Gyroscope data
    if (!sensor.getEvent(&gyro_event, Adafruit_BNO055::VECTOR_GYROSCOPE))
    {
        Serial.println("Failed to get gyroscope data");
        return;
    }

    // Gyroscope
    data_.gyro_x = gyro_event.gyro.x;
    data_.gyro_y = gyro_event.gyro.y;
    data_.gyro_z = gyro_event.gyro.z;

    // Retrieve Euler angles (Orientation)
    if (!sensor.getEvent(&euler_event, Adafruit_BNO055::VECTOR_EULER))
    {
        Serial.println("Failed to get Euler angles");
        return;
    }

    // Euler
    data_.euler_heading = euler_event.orientation.heading;
    data_.euler_roll = euler_event.orientation.roll;
    data_.euler_pitch = euler_event.orientation.pitch;

    // Retrieve Linear Acceleration data
    if (!sensor.getEvent(&linear_accel_event, Adafruit_BNO055::VECTOR_LINEARACCEL))
    {
        Serial.println("Failed to get linear acceleration data");
        return;
    }

    // Linear Acceleration
    data_.linear_accel_x = linear_accel_event.acceleration.x;
    data_.linear_accel_y = linear_accel_event.acceleration.y;
    data_.linear_accel_z = linear_accel_event.acceleration.z;

    // Retrieve Gravity Vector data
    if (!sensor.getEvent(&gravity_event, Adafruit_BNO055::VECTOR_GRAVITY))
    {
        Serial.println("Failed to get gravity vector data");
        return;
    }

    // Linear Acceleration
    data_.gravity_x = gravity_event.acceleration.x;
    data_.gravity_y = gravity_event.acceleration.y;
    data_.gravity_z = gravity_event.acceleration.z;

    // Retrieve Calibration Status
    sensor.getCalibration(&system, &gyro_cal, &accel_cal, &mag_cal);

    // Calibration
    data_.calibration_status_system = system;
    data_.calibration_status_gyro= gyro_cal;
    data_.calibration_status_accel = accel_cal;
    data_.calibration_status_mag = mag_cal;

    newDataFlag_ = true;
}

flatbuffers::Offset<SensorLog::SensorMessage> BNO055Sensor::serialize(flatbuffers::FlatBufferBuilder &builder, unsigned long timestamp) const
{
    // Create FlatBuffers MPLAltimeterData from the struct
    auto bno055 = SensorLog::CreateBNO055Data(builder, data_.accel_x, data_.accel_y, data_.accel_z, 
                                                data_.mag_x, data_.mag_y, data_.mag_z, 
                                                data_.gyro_x, data_.gyro_y, data_.gyro_z, 
                                                data_.euler_heading, data_.euler_roll, data_.euler_pitch, 
                                                data_.linear_accel_x, data_.linear_accel_y, data_.linear_accel_z, 
                                                data_.gravity_x, data_.gravity_y, data_.gravity_z, 
                                                0, 0, 0, 0);

    // Get the union offset
    auto dataOffset = bno055.Union();

    // Create the SensorMessage FlatBuffers object
    return SensorLog::CreateSensorMessage(
        builder,
        SensorLog::SensorType_BNO055,          // sensor_type
        timestamp,                             // timestamp
        SensorLog::SensorDataUnion_BNO055Data, // data_type (union type)
        dataOffset                             // data (union data)
    );
}

String BNO055Sensor::getName() const
{
    return "BNO055";
}

bool BNO055Sensor::hasNewData() const
{
    return newDataFlag_;
}

void BNO055Sensor::resetNewDataFlag()
{
    newDataFlag_ = false;
}

unsigned long BNO055Sensor::getUpdateInterval() const
{
    return 1; // Approximately 500 Hz (1000 ms / 500 ≈ 2 ms)
}
