#include <Arduino.h>

#include "sensor_manager.h"
#include "mpl_altimeter.h"
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

  // Instantiate sensor objects
  std::shared_ptr<Sensor> bno085 = std::make_shared<BNO085Sensor>();
  std::shared_ptr<Sensor> rtc = std::make_shared<RTCSensor>();
  std::shared_ptr<Sensor> mplAltimeter = std::make_shared<MPLAltimeterSensor>();

  // Add sensors to the manager
  sensorManager.addSensor(bno085);
  sensorManager.addSensor(rtc);
  sensorManager.addSensor(mplAltimeter);

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
  sensorManager.updateAll();
  sensorManager.printAllData();
  delay(1000); // Update every second
}
