// bno085.cpp
#include "bno085.h"

BNO085Sensor::BNO085Sensor() : mySensor() {}

bool BNO085Sensor::begin()
{
    // Try to initialize!
    if (!mySensor.begin_I2C())
    {
        return false;
    }

    setReports();
    return true;
}

void BNO085Sensor::setReports(void)
{
    Serial.println("Setting desired reports");
    if (!mySensor.enableReport(SH2_ACCELEROMETER))
    {
        Serial.println("Could not enable accelerometer");
    }
    if (!mySensor.enableReport(SH2_GYROSCOPE_CALIBRATED))
    {
        Serial.println("Could not enable gyroscope");
    }
    if (!mySensor.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED))
    {
        Serial.println("Could not enable magnetic field calibrated");
    }
    if (!mySensor.enableReport(SH2_LINEAR_ACCELERATION))
    {
        Serial.println("Could not enable linear acceleration");
    }
    if (!mySensor.enableReport(SH2_GRAVITY))
    {
        Serial.println("Could not enable gravity vector");
    }
    if (!mySensor.enableReport(SH2_ROTATION_VECTOR))
    {
        Serial.println("Could not enable rotation vector");
    }
    if (!mySensor.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR))
    {
        Serial.println("Could not enable geomagnetic rotation vector");
    }
    if (!mySensor.enableReport(SH2_GAME_ROTATION_VECTOR))
    {
        Serial.println("Could not enable game rotation vector");
    }
    if (!mySensor.enableReport(SH2_STEP_COUNTER))
    {
        Serial.println("Could not enable step counter");
    }
    if (!mySensor.enableReport(SH2_STABILITY_CLASSIFIER))
    {
        Serial.println("Could not enable stability classifier");
    }
    if (!mySensor.enableReport(SH2_RAW_ACCELEROMETER))
    {
        Serial.println("Could not enable raw accelerometer");
    }
    if (!mySensor.enableReport(SH2_RAW_GYROSCOPE))
    {
        Serial.println("Could not enable raw gyroscope");
    }
    if (!mySensor.enableReport(SH2_RAW_MAGNETOMETER))
    {
        Serial.println("Could not enable raw magnetometer");
    }
    if (!mySensor.enableReport(SH2_SHAKE_DETECTOR))
    {
        Serial.println("Could not enable shake detector");
    }
    if (!mySensor.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER))
    {
        Serial.println("Could not enable personal activity classifier");
    }
}

void BNO085Sensor::update()
{

    if (!mySensor.getSensorEvent(&sensorValue))
    {
        return;
    }

    switch (sensorValue.sensorId)
    {

    case SH2_GAME_ROTATION_VECTOR:
        Serial.print("Game Rotation Vector - r: ");
        Serial.print(sensorValue.un.gameRotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.gameRotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.gameRotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.gameRotationVector.k);

        sensorData_ = "Quat: " + String(sensorValue.un.gameRotationVector.real) + ", " + String(sensorValue.un.gameRotationVector.i) + ", " + String(sensorValue.un.gameRotationVector.j) + ", " + String(sensorValue.un.gameRotationVector.k);

        break;
    }
}

String BNO085Sensor::getName() const
{
    return "BNO085";
}

String BNO085Sensor::getData() const
{
    return sensorData_;
}
