#include <inttypes.h>

/// Config portion of the file
/** 
 * Define which device to build. Activate only one at a time!
 */
#define BUILD_SHINTRACKER_LEFT
//#define BUILD_SHINTRACKER_RIGHT
//#define BUILD_WIPADAPTER


/**
 * Provide required WiFi data here.
 * If you want to use the adapter make sure that WiFi configuration matches when you build 
 * the shin trackers and the adapter.
 */
#if defined(BUILD_WIPADAPTER)
const char *pSSID = "Dual-IMU-WiFi"; // provide SSID of WiFi network to connect to
const char *pPWD = "DualImuWIP"; // provide WiFi's password
#else
const char *pSSID = "Dual-IMU-WiFi"; // your WiFi's SSID
const char *pPWD = "DualImuWIP";  // matching password for your WiFi
#endif

// port to use for communication
const uint16_t Port = 10042; 

//// Code portion of the file
#if defined(BUILD_SHINTRACKER_LEFT) || defined(BUILD_SHINTRACKER_RIGHT)
#include "ShinTracker.h"

// pins to use for UART
const uint8_t PinSDA = 4;
const uint8_t PinSCL = 0;

IMUWIP::ShinTracker STracker;

void setup(void){
    // start serial
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

    // start I2C
    Wire.begin(PinSDA, PinSCL);

    #if defined(BUILD_SHINTRACKER_LEFT)
    STracker.begin(&Wire, IMUWIP::IMUPackage::DEVICE_TRACKER_LEFT, Port);
    #elif defined(BUILD_SHINTRACKER_RIGHT)
    STracker.begin(&Wire, IMUWIP::IMUPackage::DEVICE_TRACKER_RIGHT, Port);
    #endif

}//setup


void loop(void){

    STracker.update();
    delay(1);
}//loop


#elif defined(BUILD_WIPADAPTER)
#include "WIPAdapter.h"

IMUWIP::WIPAdapter Adapter;

void setup(void){
    Serial.begin(115200);

    Adapter.begin(pSSID, pPWD, Port);

}//setup

void loop(void){
    Adapter.update();

    delay(1);
}//loop


#endif

