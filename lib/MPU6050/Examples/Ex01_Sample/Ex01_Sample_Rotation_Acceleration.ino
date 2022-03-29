/*****************************************************************************\
*                                                                           *
* File(s): Ex01_Sample_Rotation_Acceleration.ino                            *
*                                                                           *
* Content: This example sketch shows how to initialize the MPU6050 and      *
*          read data from it.                                               *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#include <Arduino.h>
#include <MPU6050.h>

ArduForge::MPU6050 IMU;
/**
 * The example sketch shows how to read data from an IMU which is driven by an MPU6050 chip. The data will be written to the serial port.
 */

void setup() {
    // put your setup codes here, to run once
    Serial.begin(115200);

    // initialize IMU with default values
    IMU.begin();
}

void loop() {
    // We will simply read the dataset from the sensor and print it to the serial monitor
    ArduForge::MPU6050::SensorData Data = IMU.read();

    Serial.print("Rotation-XYZ: "); 
    Serial.print(Data.GyroX); Serial.print(" "); 
    Serial.print(Data.GyroY); Serial.print(" ");
    Serial.print(Data.GyroZ); Serial.print (" | ");
    Serial.print("Acceleration-XYZ: ");
    Serial.print(Data.AccelX); Serial.print(" ");
    Serial.print(Data.AccelY); Serial.print(" ");
    Serial.print(Data.AccelZ); Serial.print(" | ");
    // finally print the temperature information
    // note that this is the device temperature, not the room temperature
    // results may vary considerably from the readings of an MF52 thermistor.
    Serial.print(Data.Temperature); Serial.print("°C");
    Serial.println();

    delay(500);
}//loop