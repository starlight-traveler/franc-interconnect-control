// bno085.cpp
#include "bno055.h"

BNO055Sensor::BNO055Sensor() : sensor() {}

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

    // Retrieve Magnetometer data
    if (!sensor.getEvent(&mag_event, Adafruit_BNO055::VECTOR_MAGNETOMETER))
    {
        Serial.println("Failed to get magnetometer data");
        return;
    }

    // Retrieve Gyroscope data
    if (!sensor.getEvent(&gyro_event, Adafruit_BNO055::VECTOR_GYROSCOPE))
    {
        Serial.println("Failed to get gyroscope data");
        return;
    }

    // Retrieve Euler angles (Orientation)
    if (!sensor.getEvent(&euler_event, Adafruit_BNO055::VECTOR_EULER))
    {
        Serial.println("Failed to get Euler angles");
        return;
    }

    // Retrieve Linear Acceleration data
    if (!sensor.getEvent(&linear_accel_event, Adafruit_BNO055::VECTOR_LINEARACCEL))
    {
        Serial.println("Failed to get linear acceleration data");
        return;
    }

    // Retrieve Gravity Vector data
    if (!sensor.getEvent(&gravity_event, Adafruit_BNO055::VECTOR_GRAVITY))
    {
        Serial.println("Failed to get gravity vector data");
        return;
    }

    // Retrieve Calibration Status
    sensor.getCalibration(&system, &gyro_cal, &accel_cal, &mag_cal);
}

String BNO055Sensor::getName() const
{
    return "BNO055";
}

String BNO055Sensor::getData() const
{
    return sensorData_;
}

unsigned long BNO055Sensor::getUpdateInterval() const
{
    return 1; // Approximately 500 Hz (1000 ms / 500 ≈ 2 ms)
}
