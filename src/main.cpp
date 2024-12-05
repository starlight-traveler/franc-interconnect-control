#include <Arduino.h>
#include <Wire.h> // Include Wire library

#include "sensor_manager.h"
#include "mpl_altimeter.h"
#include "lsm6d032.h"
#include "bno085.h"
#include "rtc.h"
#include "bno055.h"
#include "ens160.h"
#include "bme688.h"
#include <SPI.h>
#include <SD.h>

SensorManager sensorManager(10);

void setup()
{
  Serial.begin(115200);

  if (!sensorManager.begin())
  {
    Serial.println("Failed to initialize SensorManager.");
    // Optional: Indicate error with LED
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }

  Wire.begin();          // Initialize I2C as master
  Wire.setClock(400000); // Set I2C clock to 400kHz

  // Real time clock
  // std::shared_ptr<Sensor> rtc = std::make_shared<RTCSensor>();
  // sensorManager.addSensor(rtc);

  // BNO085 Inertial measurement unit
  // std::shared_ptr<Sensor> bno085 = std::make_shared<BNO085Sensor>();
  // sensorManager.addSensor(bno085);

  // BNO055 Inertial measurement unit
  std::shared_ptr<Sensor> bno055 = std::make_shared<BNO055Sensor>();
  sensorManager.addSensor(bno055);

  // // Gas
  std::shared_ptr<Sensor> ens160 = std::make_shared<ENS160Sensor>();
  sensorManager.addSensor(ens160);

  // Temperature & Humidity

  // Temperature & Humidity & Pressure
  // std::shared_ptr<Sensor> bme688 = std::make_shared<BME688Sensor>();
  // sensorManager.addSensor(bme688);

  // Alitmeter
  std::shared_ptr<Sensor> mplAltimeter = std::make_shared<MPLAltimeterSensor>();
  sensorManager.addSensor(mplAltimeter);

  // 32g Accelerometer
  std::shared_ptr<Sensor> lsm32d032 = std::make_shared<LSM6D032Sensor>();
  sensorManager.addSensor(lsm32d032);

  // Initialize all sensors
  if (!sensorManager.beginAll())
  {
    Serial.println("One or more sensors failed to initialize!");
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }
  else
  {
    Serial.println("All sensors initialized successfully.");
  }

  // After setup is passed, turn on the built-in LED
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{

  sensorManager.updateAll();
  sensorManager.logAllData();

  // // Retrieve and print the update rate
  // float updateRate = sensorManager.getUpdateRateHz();
  // if (updateRate > 0.0f)
  // {
  //   Serial.print("Update Rate: ");
  //   Serial.print(updateRate, 2); // Print with 2 decimal places
  //   Serial.println(" Hz");
  // }

}

// void loop()
// {
//   sensorManager.updateAll();
//   sensorManager.printAllData();
// }
