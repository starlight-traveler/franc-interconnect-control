#include <Arduino.h>
#include <Wire.h> // Include Wire library

#include "sensor_manager.h"
#include "mpl_altimeter.h"
#include "lsm6d032.h"
#include "bno085.h"
#include "rtc.h"

SensorManager sensorManager;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  } // Wait for serial port to connect

  Wire.begin();          // Initialize I2C as master
  Wire.setClock(400000); // Set I2C clock to 400kHz

  // Instantiate sensor objects
  std::shared_ptr<Sensor> bno085 = std::make_shared<BNO085Sensor>();
  std::shared_ptr<Sensor> rtc = std::make_shared<RTCSensor>();
  std::shared_ptr<Sensor> mplAltimeter = std::make_shared<MPLAltimeterSensor>();
  std::shared_ptr<Sensor> lsm32d032 = std::make_shared<LSM6D032Sensor>();

  // Add sensors to the manager
  sensorManager.addSensor(bno085);
  sensorManager.addSensor(mplAltimeter);
  sensorManager.addSensor(lsm32d032);

  // sensorManager.addSensor(rtc);

  // Initialize all sensors
  if (!sensorManager.beginAll())
  {
    Serial.println("One or more sensors failed to initialize!");
  }
  else
  {
    Serial.println("All sensors initialized successfully.");
  }
}

void loop()
{
  sensorManager.updateAllWithHz();

  // Retrieve and print the update rate
  float updateRate = sensorManager.getUpdateRateHz();
  if (updateRate > 0.0f)
  {
    Serial.print("Update Rate: ");
    Serial.print(updateRate, 2); // Print with 2 decimal places
    Serial.println(" Hz");
  }

  // Delay to prevent flooding the serial monitor
  // Adjust as needed based on your requirements
  // delay(100); // Example: 100ms delay
}

// void loop()
// {
//   sensorManager.updateAll();
//   sensorManager.printAllData();
// }
