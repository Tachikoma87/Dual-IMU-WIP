#include <Arduino.h>

#if defined(ESP32)
#include <WiFi.h> // header for ESP32
#elif defined(ESP8266)
#include <ESP8266WiFi.h> // header for ESP8266
#endif

#include <WiFiUdp.h>
#include <MPU6050.h>
#include "IMUPackage.hpp"
#include "IMUDataAverager.hpp"
#include "WiFiData.h"

// build for left or for right leg?
const bool BuildLeftLeg = true;

const char *pUDPTargetAddress = "192.168.1.206"; // IP address of host/target system
const uint16_t UDPPort = (BuildLeftLeg) ?  25000 : 25001;
const uint8_t PinSDA = 4;
const uint8_t PinSCL = 0;

// create UDP instance
WiFiUDP Udp;
// create sensor instance
ArduForge::MPU6050 IMU;

uint32_t StartTime;

uint8_t Buffer[256];

uint32_t LastPackage = 0;
uint32_t LastIMURead = 0;

uint32_t IMUSamplingFreq = 10; // every 10 milliseconds
uint32_t PackageSendingFreq = 20; // every 20 milliseconds

IMUWIP::IMUDataAverager DataAvg;

void setup() {
    Serial.begin(115200);
    // connect to the WiFi network
    WiFi.begin(pSSID, pPWD);
    Serial.print("Connecting to WiFi .");
    while(WiFi.status() != WL_CONNECTED){
        delay(250);
        Serial.print(".");
    }
    Serial.print(" Connected to \n");
    Serial.print("\tNetwork: "); Serial.print(pSSID); Serial.print("\n");
    Serial.print("\tIP: "); Serial.print(WiFi.localIP());Serial.print("\n");
  
    StartTime = millis();

    Udp.begin(27341);
    Wire.begin(PinSDA, PinSCL);
    IMU.begin();
    IMU.calibrate();

    DataAvg.begin(30, 200);
}//setup

void loop() {

    if(millis() - LastIMURead > IMUSamplingFreq){
        // read IMU Data
        DataAvg.addData(IMU.read());
        LastIMURead = millis();
    }

    if(millis() - LastPackage > PackageSendingFreq){

        // send UDP package
        IPAddress Target;
        Target.fromString(pUDPTargetAddress);

        // read IMU data
        ArduForge::MPU6050::SensorData Data; // = IMU.read();
        DataAvg.retrieveData(&Data);
        ArduForge::IMUPackage Package;
        Package.GyroX = Data.GyroX;
        Package.GyroY = Data.GyroY;
        Package.GyroZ = Data.GyroZ;
        Package.AccelX = Data.AccelX;
        Package.AccelY = Data.AccelY;
        Package.AccelZ = Data.AccelZ;
        Package.Cmd = ArduForge::IMUPackage::CMD_AVG_DATA;

        // pack to stream and send
        uint16_t MsgSize = Package.toStream(Buffer, sizeof(Buffer));
        Udp.beginPacket(Target, UDPPort);
        Udp.write(Buffer, MsgSize);
        Udp.endPacket();

        LastPackage = millis();
    }

    // Did we receive data?
    Udp.parsePacket();
    int16_t MsgSize = Udp.read(Buffer, sizeof(Buffer));

    if(MsgSize > 0){
         if(ArduForge::IMUPackage::checkMagicTag(Buffer)){
             ArduForge::IMUPackage P;
             P.fromStream(Buffer, MsgSize);

             if(P.Cmd == ArduForge::IMUPackage::CMD_CALIBRATE){
                 IMU.calibrate();
             } 
         }//if[IMUPackage]
    }//if[received message]

    delay(1);
}//loop