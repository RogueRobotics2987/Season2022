//THIS CODE MUST BE RAN ON THE ARDUINO IDE TO OUTPUT THE ENCODED RANGE

/*
This code takes range measurements with the VL53L1X and displays range in mm for each measurement, which can be decoded in the drivetrain.cpp file.
*/

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(15000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(15);
}

void loop()
{
//  static long int lastTime = 0;
//  long int curTime = millis();
//  Serial.print(",");
//  Serial.print(curTime - lastTime);
//  Serial.print(",");
//  lastTime = curTime;

//  long int startTime = millis();
  sensor.read();
//  long int stopTime = millis();
//  Serial.print(",");
//  Serial.print(stopTime - startTime);
//  Serial.print(",");
  
  
  Serial.print("B");
  Serial.print(sensor.ranging_data.range_mm);
  Serial.print("E");
  //Serial.print("\tstatus: ");
  //Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  //Serial.print("\tpeak signal: ");
  //Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  //Serial.print("\tambient: ");
  //Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);
  
  Serial.println();
}
