// MPLAltimeterSensor.cpp
#include "lsm6d032.h"

LSM6D032Sensor::LSM6D032Sensor() : lsm() {}

bool LSM6D032Sensor::begin()
{
    Wire.begin();
    if (!lsm.begin_I2C())
    {
        // Serial.println("LSM6D032 initialization failed!");
        return false;
    }

    // Serial.println("LSM6D032 initialized.");
    lsm.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    lsm.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

    return true;
}

void LSM6D032Sensor::update()
{
    // Implement timing check
    unsigned long currentTime = millis();
    unsigned long desiredInterval = getUpdateInterval();
    if (currentTime - lastUpdateTime >= desiredInterval)
    {
        lsm.getEvent(&accel, &gyro, &temp);


        // Acceleration
        float accel_x = accel.acceleration.x;
        float accel_y = accel.acceleration.y;
        float accel_z = accel.acceleration.z;

        // Gyroscope
        float gyro_x = gyro.gyro.x;
        float gyro_y = gyro.gyro.y;
        float gyro_z = gyro.gyro.z;

        sensorData_ = "placeholder";
        lastUpdateTime = currentTime;
    }
}

String LSM6D032Sensor::getName() const
{
    return "LSM6D032";
}

String LSM6D032Sensor::getData() const
{
    return sensorData_;
}

unsigned long LSM6D032Sensor::getUpdateInterval() const
{
    return 40; // Approximately 30 Hz (1000 ms / 30 ≈ 33 ms)
}