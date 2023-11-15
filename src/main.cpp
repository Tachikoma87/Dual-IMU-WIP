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
const char *pSSID = "TP-Link_272C"; // provide SSID of WiFi network to connect to
const char *pPWD = "06311261"; // provide WiFi's password
const bool ConnectWiFi = false; // Switch to connect to WiFi or not

// build for left or for right leg?
const bool BuildRightLeg = true;

const char *pUDPTargetAddress = "192.168.1.206"; // IP address of host/target system
const uint16_t UDPPort = (BuildRightLeg) ?  25000 : 25001;
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

//global variables
// Use the following global variables and access functions to help store the overall
unsigned long last_read_time;

//void set_last_read_angle_data(unsigned long Time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
void set_last_read_angle_data(unsigned long Time) {  
  last_read_time = Time;
}

inline unsigned long get_last_time() {return last_read_time;}

// Function to calculate IMU offsets
void calculateIMUOffsets(float &gyroOffsetX, float &gyroOffsetY, float &gyroOffsetZ, 
                         float &accelOffsetX, float &accelOffsetY, float &accelOffsetZ) {
    const int samples_n = 1000;
    float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
    float accelSumX = 0, accelSumY = 0, accelSumZ = 0;

    for (int i = 0; i < samples_n; ++i) {
        // Assuming IMU.read() returns a structure with gyro and accel data
        auto data = IMU.read(); 
        gyroSumX += data.GyroX;
        gyroSumY += data.GyroY;
        gyroSumZ += data.GyroZ;
        accelSumX += data.AccelX;
        accelSumY += data.AccelY;
        accelSumZ += data.AccelZ;
        delay(10); // Small delay between readings
    }

    gyroOffsetX = gyroSumX / samples_n;
    gyroOffsetY = gyroSumY / samples_n;
    gyroOffsetZ = gyroSumZ / samples_n;
    accelOffsetX = accelSumX / samples_n;
    accelOffsetY = accelSumY / samples_n;
    accelOffsetZ = accelSumZ / samples_n;
}



IMUWIP::IMUDataAverager DataAvg;

// Global variables for offsets
float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
float accelOffsetX, accelOffsetY, accelOffsetZ;

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
    
     // Calculate offsets
    calculateIMUOffsets(gyroOffsetX, gyroOffsetY, gyroOffsetZ, 
                        accelOffsetX, accelOffsetY, accelOffsetZ);

    set_last_read_angle_data(millis());

    // begin communication with TOF sensor
    if(!TOFSensor.begin()){
        Serial.print("Error initializing VL53L0X!\n");
    }

    DataAvg.begin(30, 200);
}//setup

int steps = 0;
float mag;
float prev = 0;
float threshold =0.3; 
float AccelAngleX=0, AccelAngleY=0, AccelAngleZ=0;
double dt, Previous_Time, Time;
float d, h;

// main loop
void loop() {
    
    // process sensor data?
    if(millis() - LastIMURead > IMUSamplingFreq){
        // read IMU Data
        DataAvg.addData(IMU.read());
        LastIMURead = millis();
        // retrieve averaged IMU data
        ArduForge::MPU6050::SensorData IMUSensorData;
        DataAvg.retrieveData(&IMUSensorData);
        IMUSensorData.GyroX -= gyroOffsetX;
        IMUSensorData.GyroY -= gyroOffsetY;
        IMUSensorData.GyroZ -= gyroOffsetZ;
        IMUSensorData.AccelX -= accelOffsetX;
        IMUSensorData.AccelY -= accelOffsetY;
        IMUSensorData.AccelZ -= accelOffsetZ;

        DataAvg.addData(IMUSensorData);
        LastIMURead = millis();
    }//if[average IMU data]

    if(millis() - LastSensorPrint > SensorsPrintInterval){

        // retrieve averaged IMU data
        ArduForge::MPU6050::SensorData IMUSensorData;
        DataAvg.retrieveData(&IMUSensorData);

        // average Acceleration data X,Y,Z
        mag = sqrt((IMUSensorData.AccelX*IMUSensorData.AccelX)+(IMUSensorData.AccelY*IMUSensorData.AccelY)+(IMUSensorData.AccelZ*IMUSensorData.AccelZ));
        
        // for walking
        if (mag>=threshold && prev<threshold)
        {
            steps+=1;
        }
        prev = mag;

        // Get angle values from IMU
        float RADIANS_TO_DEGREES = 180/3.14159;
        // float AccelAngleY = atan(-1*IMUSensorData.AccelY/sqrt(pow(IMUSensorData.AccelZ,2) + pow(IMUSensorData.AccelX,2)))*RADIANS_TO_DEGREES;
        // float AccelAngleZ = atan(IMUSensorData.AccelZ/sqrt(IMUSensorData.AccelY))*RADIANS_TO_DEGREES;

        //float AccelAngleY = atan(IMUSensorData.AccelY/sqrt((IMUSensorData.AccelZ*IMUSensorData.AccelZ) + (IMUSensorData.AccelX*IMUSensorData.AccelX)))*RADIANS_TO_DEGREES;
        //float AccelAngleZ = atan(-1*IMUSensorData.AccelZ/sqrt((IMUSensorData.AccelY*IMUSensorData.AccelY) + (IMUSensorData.AccelX*IMUSensorData.AccelX)))*RADIANS_TO_DEGREES;
        //float AccelAngleX = atan((sqrt(IMUSensorData.AccelY*IMUSensorData.AccelY) + (IMUSensorData.AccelZ*IMUSensorData.AccelZ))/IMUSensorData.AccelX)*RADIANS_TO_DEGREES;
            //float AccelAngleX = 0;

        float AccelAngleY = atan2(IMUSensorData.AccelY, sqrt(pow(IMUSensorData.AccelZ, 2) + pow(IMUSensorData.AccelX, 2))) * RADIANS_TO_DEGREES;
        float AccelAngleX = atan2(IMUSensorData.AccelX, sqrt(pow(IMUSensorData.AccelZ, 2) + pow(IMUSensorData.AccelY, 2))) * RADIANS_TO_DEGREES;


        Previous_Time = Time;// the previous time is stored before the actual time read
        Time = millis();// actual time read
        float dt = (Time - Previous_Time) / 1000;
        float GyroAngleY = IMUSensorData.GyroY*dt;
        float GyroAngleZ = IMUSensorData.GyroZ*dt;
        float GyroAngleX = IMUSensorData.GyroX*dt;

           // Compute the drifting gyro angles
        //float unfiltered_GyroAngleY = IMUSensorData.GyroY*dt + get_last_gyro_y_angle();s
        //float unfiltered_GyroAngleZ = IMUSensorData.GyroZ*dt + get_last_gyro_z_angle();
        //float unfiltered_GyroAngleX = IMUSensorData.GyroX*dt + get_last_gyro_x_angle();
 
        // // Apply the complementary filter to figure out the change in angle - choice of alpha is estimated now.  Alpha depends on the sampling rate...
        float alpha = 0.04;
        float angle_x = alpha*GyroAngleX + (1.0 - alpha)*AccelAngleX;
        float angle_y = alpha*GyroAngleY + (1.0 - alpha)*AccelAngleY;
        //float angle_z = GyroAngleZ;  //Accelerometer doesn't give z-angle

         // Update the saved data with the latest values
        //set_last_read_time(Time);

        // perform distance measurement
        VL53L0X_RangingMeasurementData_t DistMeasurement;
        TOFSensor.rangingTest(&DistMeasurement);
        float Dist = DistMeasurement.RangeMilliMeter/10.0f;

        // When standing Dist = h;

        // while walking       
        float a_dash = 90 - AccelAngleX; // α' = 90 - pitch angle
        float h = Dist * sin(a_dash * PI / 180);

        // Calculate foot angle (β) using height and distance
        float beta = (h <= Dist) ? acos(h / Dist) * RADIANS_TO_DEGREES : 0.0f; // Convert to degrees

        Serial.printf("IMU sensor data:\n");
        //Serial.printf("\t Gyro(y|z|x): %.2f | %.2f | %.2f\n", IMUSensorData.GyroX, IMUSensorData.GyroY, IMUSensorData.GyroZ);
        Serial.printf("\t Acceleration(x|y|z): %.2f | %.2f | %.2f\n", IMUSensorData.AccelX, IMUSensorData.AccelY, IMUSensorData.AccelZ);
        Serial.printf("\t Average Acceleration Data : %.2f\n", mag );
        Serial.printf("\t Steps : %.2d\n", steps);

        
        Serial.printf("\t Acceleration Angle(y|z|x): %.2f | %.2f | %.2f\n", AccelAngleY, AccelAngleZ, AccelAngleX);
        //Serial.printf("\t Gyroscope Angle(y|z|x): %.2f | %.2f | %.2f\n", GyroAngleY, GyroAngleZ, GyroAngleX);
        //Serial.printf("\t Angle(y|z|x): %.2f | %.2f | %.2f\n", angle_y, angle_z, angle_x);
        Serial.printf("TOF sensor data:\n");
        Serial.printf("\t Distance: %.2f cm\n", Dist);
        Serial.printf("\t TOF Sensor Tilt Angle: %.2f degree\n", a_dash);
        Serial.printf("\t Height Estimation from ground source: %.2f cm\n", h );
        Serial.printf("\t Foot Angle: %.2f degree\n", beta);

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
