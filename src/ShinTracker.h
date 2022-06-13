/*****************************************************************************\
*                                                                           *
* File(s): ShinTracker.h and ShinTracker.cpp                         *
*                                                                           *
* Content: Tracks the data of an IMU placed on the shin of a person.   *
*                       *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* FreeBSD License without any warranty or guaranty to work properly.        *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __IMUWIP_SHINTRACKER_H__
#define __IMUWIP_SHINTRAKCER_H__


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

namespace IMUWIP{
    class ShinTracker{
        public:
            ShinTracker(void);

            ~ShinTracker(void);

            void begin(TwoWire *pWire, IMUPackage::DeviceType Type, const uint16_t Port = 10042);
            void end(void);
            void update(void);

        private:
            // create UDP instance
            WiFiUDP m_Socket; // Udp socket
            // create sensor instance
            ArduForge::MPU6050 m_IMU;

            uint32_t m_StartTime;

            uint8_t m_Buffer[64];

            uint32_t m_LastPackage = 0;
            uint32_t m_LastIMURead = 0;

            uint32_t m_IMUSamplingIntervall = 10; // every 10 milliseconds
            uint32_t m_PackageSendingIntervall = 20; // every 20 milliseconds

            TwoWire *m_pWire;

            IMUDataAverager m_DataAvg;

            IPAddress m_TargetIP;
            uint16_t m_TargetPort;

            IMUPackage::DeviceType m_Type; // own type (left or right tracker)
    
    };//ShinTracker

}//name space


#endif 