#include <Arduino.h>

#if defined(ESP32)
#include <WiFi.h> // header for ESP32
#elif defined(ESP8266)
#include <ESP8266WiFi.h> // header for ESP8266
#endif

#include <WiFiUdp.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>
#include "IMUPackage.hpp"
#include "IMUDataAverager.hpp"

// WiFi credentials (currently not required on this branch)
const char *pSSID = "..."; // provide SSID of WiFi network to connect to
const char *pPWD = "..."; // provide WiFi's password
const bool ConnectWiFi = false; // Switch to connect to WiFi or not

// build for left or for right leg?
const bool BuildLeftLeg = true;

const char *pUDPTargetAddress = "192.168.1.206"; // IP address of host/target system
const uint16_t UDPPort = (BuildLeftLeg) ?  25000 : 25001;
const uint8_t PinSDA = 4;
const uint8_t PinSCL = 0;

// create UDP instance
WiFiUDP Udp;
// create sensor instances
ArduForge::MPU6050 IMU;
Adafruit_VL53L0X TOFSensor = Adafruit_VL53L0X();

uint32_t StartTime;

uint8_t Buffer[256];

uint32_t LastPackage = 0;
uint32_t LastIMURead = 0;

uint32_t IMUSamplingFreq = 10; // every 10 milliseconds
uint32_t PackageSendingFreq = 20; // every 20 milliseconds

IMUWIP::IMUDataAverager DataAvg;


const uint32_t SensorsPrintInterval = 1500; // print interval in milliseconds
uint32_t LastSensorPrint = 0;

void setup() {
    Serial.begin(115200);
    // connect to the WiFi network
    if(ConnectWiFi){
        WiFi.begin(pSSID, pPWD);
        Serial.print("Connecting to WiFi .");
        while(WiFi.status() != WL_CONNECTED){
            delay(250);
            Serial.print(".");
        }
        Serial.print(" Connected to \n");
        Serial.print("\tNetwork: "); Serial.print(pSSID); Serial.print("\n");
        Serial.print("\tIP: "); Serial.print(WiFi.localIP());Serial.print("\n");

         Udp.begin(27341);
    }//if[connect WiFi]
    
    StartTime = millis();

    // start I2C
    Wire.begin(PinSDA, PinSCL);

    // begin communication with IMU
    IMU.begin();
    IMU.calibrate();

    // begin communication with TOF sensor
    if(!TOFSensor.begin()){
        Serial.print("Error initializing VL53L0X!\n");
    }

    DataAvg.begin(30, 200);
}//setup

// main loop
void loop() {
    
    // process sensor data?
    if(millis() - LastIMURead > IMUSamplingFreq){
        // read IMU Data
        DataAvg.addData(IMU.read());
        LastIMURead = millis();
    }//if[average IMU data]

    if(millis() - LastSensorPrint > SensorsPrintInterval){

        // retrieve averaged IMU data
        ArduForge::MPU6050::SensorData IMUSensorData;
        DataAvg.retrieveData(&IMUSensorData);

        // perform distance measurement
        VL53L0X_RangingMeasurementData_t DistMeasurement;
        TOFSensor.rangingTest(&DistMeasurement);

        Serial.printf("IMU sensor data:\n");
        Serial.printf("\t Gyro(x|y|z): %.2f | %.2f | %.2f\n", IMUSensorData.GyroX, IMUSensorData.GyroY, IMUSensorData.GyroZ);
        Serial.printf("\t Acceleration(x|y|z): %.2f | %.2f | %.2f\n", IMUSensorData.AccelX, IMUSensorData.AccelY, IMUSensorData.AccelZ);
        Serial.printf("TOF sensor data:\n");
        Serial.printf("\tDistance: %.2f cm\n", DistMeasurement.RangeMilliMeter/10.0f);

        LastSensorPrint = millis();
    }//if[print sensor data]

    if(ConnectWiFi){
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
    }//if[connect WiFi]

    delay(1);
}//loop