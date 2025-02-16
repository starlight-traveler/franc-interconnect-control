#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

#include "sensor_manager.h"
#include "mpl_altimeter.h"
#include "lsm6d032.h"
#include "bno085.h"
#include "rtc.h"
#include "bno055.h"
#include "ens160.h"
#include "bme688.h"
#include "master_sensor_struct.h" // Defines MasterSensorData

// Create a SensorManager using SD chip select pin 10
SensorManager sensorManager(10);

// Global shared pointer
std::shared_ptr<Sensor> mplAltimeter;

// Define a maximum allowed sensor update duration (in ms)
const unsigned long SENSOR_UPDATE_TIMEOUT_MS = 5000;

// ---------------------------------------------------------------------
// Compute CRC32 using Ethernet polynomial (0xEDB88320)
// ---------------------------------------------------------------------
uint32_t computeCRC32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      uint32_t mask = (crc & 1) ? 0xEDB88320 : 0;
      crc = (crc >> 1) ^ mask;
    }
  }
  return ~crc;
}

// ---------------------------------------------------------------------
// performHandshake()
// Waits for a "HELLO" from the host and responds with "ACKHELLO".
// Returns true if handshake was successful.
// ---------------------------------------------------------------------
bool performHandshake()
{
  Serial.println("Waiting for handshake from host...");
  unsigned long handshakeTimeout = 10000; // 10 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < handshakeTimeout)
  {
    if (Serial.available() > 0)
    {
      String incoming = Serial.readStringUntil('\n');
      incoming.trim(); // Remove extra whitespace/newlines
      if (incoming.equals("HELLO"))
      {
        Serial.println("ACKHELLO");
        return true;
      }
    }
  }
  return false;
}

// ---------------------------------------------------------------------
// processAndSendJSON()
// - Aggregates sensor data into a JSON object,
// - Computes a CRC over the JSON (excluding the CRC field),
// - Adds the CRC field, serializes the final JSON, and sends it.
// ---------------------------------------------------------------------
void processAndSendJSON(const MasterSensorData &data)
{
  StaticJsonDocument<1024> doc;

  // Populate JSON with aggregated sensor data:
  doc["timestamp"] = data.timestamp;
  doc["bme_temperature"] = data.bme_temperature;
  doc["bme_pressure"] = data.bme_pressure;
  doc["bme_humidity"] = data.bme_humidity;
  doc["bme_gas_resistance"] = data.bme_gas_resistance;
  doc["bme_altitude"] = data.bme_altitude;

  doc["ens_aqi"] = data.ens_aqi;
  doc["ens_tvoc"] = data.ens_tvoc;
  doc["ens_eco2"] = data.ens_eco2;
  doc["ens_hp0"] = data.ens_hp0;
  doc["ens_hp1"] = data.ens_hp1;
  doc["ens_hp2"] = data.ens_hp2;
  doc["ens_hp3"] = data.ens_hp3;

  doc["lsm_accel_x"] = data.lsm_accel_x;
  doc["lsm_accel_y"] = data.lsm_accel_y;
  doc["lsm_accel_z"] = data.lsm_accel_z;
  doc["lsm_gyro_x"] = data.lsm_gyro_x;
  doc["lsm_gyro_y"] = data.lsm_gyro_y;
  doc["lsm_gyro_z"] = data.lsm_gyro_z;

  doc["mpl_pressure"] = data.mpl_pressure;
  doc["mpl_altitude"] = data.mpl_altitude;

  doc["bno_accel_x"] = data.bno_accel_x;
  doc["bno_accel_y"] = data.bno_accel_y;
  doc["bno_accel_z"] = data.bno_accel_z;
  doc["bno_mag_x"] = data.bno_mag_x;
  doc["bno_mag_y"] = data.bno_mag_y;
  doc["bno_mag_z"] = data.bno_mag_z;
  doc["bno_gyro_x"] = data.bno_gyro_x;
  doc["bno_gyro_y"] = data.bno_gyro_y;
  doc["bno_gyro_z"] = data.bno_gyro_z;
  doc["bno_euler_heading"] = data.bno_euler_heading;
  doc["bno_euler_roll"] = data.bno_euler_roll;
  doc["bno_euler_pitch"] = data.bno_euler_pitch;
  doc["bno_linear_accel_x"] = data.bno_linear_accel_x;
  doc["bno_linear_accel_y"] = data.bno_linear_accel_y;
  doc["bno_linear_accel_z"] = data.bno_linear_accel_z;
  doc["bno_gravity_x"] = data.bno_gravity_x;
  doc["bno_gravity_y"] = data.bno_gravity_y;
  doc["bno_gravity_z"] = data.bno_gravity_z;
  doc["bno_calibration_system"] = data.bno_calibration_system;
  doc["bno_calibration_gyro"] = data.bno_calibration_gyro;
  doc["bno_calibration_accel"] = data.bno_calibration_accel;
  doc["bno_calibration_mag"] = data.bno_calibration_mag;

  // Serialize JSON without the CRC field.
  char jsonBuffer[1024];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // Compute CRC32 over the JSON string (without the CRC field)
  uint32_t crc = computeCRC32((const uint8_t *)jsonBuffer, strlen(jsonBuffer));

  // Add the CRC field.
  doc["crc"] = crc;

  // Serialize the final JSON with the CRC field.
  char finalJson[1024];
  serializeJson(doc, finalJson, sizeof(finalJson));

  // Send the JSON over Serial.
  Serial.println(finalJson);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for Serial to initialize

  // // Perform handshake with the host.
  // if (!performHandshake())
  // {
  //   Serial.println("Handshake failed. Halting.");
  //   while (true)
  //   {
  //     digitalWrite(LED_BUILTIN, HIGH);
  //     delay(250);
  //     digitalWrite(LED_BUILTIN, LOW);
  //     delay(250);
  //   }
  // }

  if (!sensorManager.begin())
  {
    Serial.println("Failed to initialize SensorManager.");
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }

  // Initialize I2C and set clock speed.
  Wire.begin();
  Wire.setClock(400000);

  // Add sensors.
  std::shared_ptr<Sensor> bno055 = std::make_shared<BNO055Sensor>();
  sensorManager.addSensor(bno055);

  std::shared_ptr<Sensor> ens160 = std::make_shared<ENS160Sensor>();
  sensorManager.addSensor(ens160);

  mplAltimeter = std::make_shared<MPLAltimeterSensor>();
  sensorManager.addSensor(mplAltimeter);

  std::shared_ptr<Sensor> lsm32d032 = std::make_shared<LSM6D032Sensor>();
  sensorManager.addSensor(lsm32d032);

  // Initialize all sensors.
  if (!sensorManager.beginAll())
  {
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }
}

void loop()
{

  /**
   * @brief Rocket is on the launchpad/in-flight
   */
  while (true) {

    // Update only the altimeter sensor.
    mplAltimeter->update();

    // If new altimeter data is available, read and send it.
    if (mplAltimeter->hasNewData())
    {
      // Retrieve the altimeter data and cast it appropriately.
      const SensorData *data = mplAltimeter->getData();
      const MPLAltimeterDataStruct *altData = static_cast<const MPLAltimeterDataStruct *>(data);

      Serial.print("SensorData: ");
      Serial.println(altData->altitude);

      mplAltimeter->resetNewDataFlag();

      delay(1);
    }
  }

  /**
   * @brief Rocket has landed
   */
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equals("SEND"))
    {
      // Update sensors.
      sensorManager.updateAll();

      // Aggregate sensor data into a master structure.
      MasterSensorData masterData = sensorManager.getAggregatedData();

      // Create and send JSON from the master data.
      processAndSendJSON(masterData);

      // Wait for an ACK from the host before looping again.
      unsigned long ackStart = millis();
      bool ackReceived = false;
      while (millis() - ackStart < 5000)
      { // 5-second timeout
        if (Serial.available() > 0)
        {
          String ack = Serial.readStringUntil('\n');
          ack.trim();
          if (ack.equals("ACK"))
          {
            ackReceived = true;
            break;
          }
        }
      }
      if (!ackReceived)
      {
        Serial.println("No ACK received from host.");
      }
    }
  }
  delay(100); // Small delay to avoid busy looping.
}
