/*****************************************************************************\
*                                                                           *
* File(s): WIPAdapter.h and WIPAdapter.cpp                                  *
*                                                                           *
* Content: Creates a WiFi access point and searches for shin trackers.      *
*          Provides the current accelerometer and gyroscope data through    *
*          UART in regular interval.                                        *
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
#ifndef __IMUWIP_WIPADAPTER_H__
#define __IMUWIP_WIPADAPTER_H__

#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h> // header for ESP32
#elif defined(ESP8266)
#include <ESP8266WiFi.h> // header for ESP8266
#endif

#include <WiFiUdp.h>
#include "IMUPackage.hpp"

namespace IMUWIP{

    /***
     * \brief This adapter creates a local WiFi network to which the shin-trackers can connect. Data will be printed to serial.
     * 
     */
    class WIPAdapter{
        public:

            const uint8_t LEFT = 0;
            const uint8_t RIGHT = 1;

            WIPAdapter(void);
            ~WIPAdapter(void);

            void begin(const char *pSSID, const char *pPWD, const uint16_t Port);
            void end(void);

            void update(void);
            void printCurrentToSerial(void);
        private:


            IPAddress m_ShinTrackersIP[2];
            uint16_t m_ShinTrackersPort[2];
            
            uint32_t m_LastSearch;
            uint32_t m_LastPrint;

            WiFiUDP m_Socket;
            uint16_t m_UDPPort;

            uint8_t m_Buffer[64];

            IPAddress m_LocalIP;
            IPAddress m_BroadcastIP;
            IPAddress m_GatewayIP;

            IMUPackage m_DataLeft;
            IMUPackage m_DataRight;

    };//WIPAdapter

}//name space


#endif 