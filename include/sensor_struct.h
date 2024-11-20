#ifndef SENSOR_STRUCT_H
#define SENSOR_STRUCT_H

#include <stdint.h>

struct BME688DataStruct
{
    float temperature;    // Temperature in Celsius
    float pressure;       // Pressure in hPa
    float humidity;       // Humidity in %
    float gas_resistance; // Gas resistance in KOhms
    float altitude;       // Altitude in meters
};

struct ENS160DataStruct
{
    int aqi;   // Air Quality Index
    int tvoc;  // Total Volatile Organic Compounds in ppb
    int eco2;  // Estimated CO2 in ppm
    float hp0; // Hotplate resistance HP0 in Ohms
    float hp1; // Hotplate resistance HP1 in Ohms
    float hp2; // Hotplate resistance HP2 in Ohms
    float hp3; // Hotplate resistance HP3 in Ohms
};

struct LSM6D032DataStruct
{
    float accel_x; // Acceleration in x-axis (m/s²)
    float accel_y; // Acceleration in y-axis (m/s²)
    float accel_z; // Acceleration in z-axis (m/s²)
    float gyro_x;  // Gyroscope in x-axis (°/s)
    float gyro_y;  // Gyroscope in y-axis (°/s)
    float gyro_z;  // Gyroscope in z-axis (°/s)
};

struct MPLAltimeterDataStruct
{
    float pressure; // Atmospheric pressure in Pa
    float altitude; // Altitude in meters
};

struct BNO055DataStruct
{
    float accel_x;                     // Acceleration in x-axis (m/s²)
    float accel_y;                     // Acceleration in y-axis (m/s²)
    float accel_z;                     // Acceleration in z-axis (m/s²)
    float mag_x;                       // Magnetic field in x-axis (μT)
    float mag_y;                       // Magnetic field in y-axis (μT)
    float mag_z;                       // Magnetic field in z-axis (μT)
    float gyro_x;                      // Gyroscope in x-axis (°/s)
    float gyro_y;                      // Gyroscope in y-axis (°/s)
    float gyro_z;                      // Gyroscope in z-axis (°/s)
    float euler_heading;               // Heading in Euler angles (°)
    float euler_roll;                  // Roll in Euler angles (°)
    float euler_pitch;                 // Pitch in Euler angles (°)
    float linear_accel_x;              // Linear acceleration in x-axis (m/s²)
    float linear_accel_y;              // Linear acceleration in y-axis (m/s²)
    float linear_accel_z;              // Linear acceleration in z-axis (m/s²)
    float gravity_x;                   // Gravity vector in x-axis (m/s²)
    float gravity_y;                   // Gravity vector in y-axis (m/s²)
    float gravity_z;                   // Gravity vector in z-axis (m/s²)
    uint8_t calibration_status_system; // System calibration status (0-3)
    uint8_t calibration_status_gyro;   // Gyro calibration status (0-3)
    uint8_t calibration_status_accel;  // Accelerometer calibration status (0-3)
    uint8_t calibration_status_mag;    // Magnetometer calibration status (0-3)
};

#endif // SENSOR_STRUCT_H
