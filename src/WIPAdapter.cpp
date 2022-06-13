#include "WIPAdapter.h"

namespace IMUWIP{

    WIPAdapter::WIPAdapter(void){
        m_ShinTrackersPort[0] = 0;
        m_ShinTrackersPort[1] = 0;
        m_LastSearch = 0;
        m_LastPrint = 0;
        m_UDPPort = 0;
    }//Constructor

    WIPAdapter::~WIPAdapter(void){

    }//Destructor

    void WIPAdapter::begin(const char *pSSID, const char *pPWD, const uint16_t Port){
        // config WiFi
        m_LocalIP.fromString("192.168.4.2");
        m_BroadcastIP.fromString("192.168.4.255");
        m_GatewayIP.fromString("192.168.4.1");
        IPAddress subnet(255,255,255,0);
        // create access point
        WiFi.softAPConfig(m_LocalIP, m_GatewayIP, subnet);

        if(!WiFi.softAP(pSSID, pPWD)){
            Serial.print("Error creating WiFi access point!\n");
        }else{
            Serial.print("WiFi access point created!\n");
        }

        Serial.print("\nWifi ready!\n");
        Serial.printf("\t Local IP: %s\n", WiFi.softAPIP().toString().c_str());
        Serial.printf("\t Gateway IP: %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("\t Broadcast IP: %s\n", m_BroadcastIP.toString().c_str());

        m_UDPPort = Port;
        if(0 == m_Socket.begin(m_UDPPort)) Serial.print("Creating socket failed!\n");
    }//begin

    void WIPAdapter::end(void){
        WiFi.disconnect(true);
    }//end

    void WIPAdapter::update(void){
        // search for 
        if((m_ShinTrackersPort[LEFT] == 0 || m_ShinTrackersPort[RIGHT] == 0) && millis() - m_LastSearch > 1000){
            // search for shin trackers
            IMUPackage Pack;
            Pack.Cmd = IMUPackage::CMD_SEARCH;
            // set own ip
            Pack.setIP(WiFi.softAPIP().toString().c_str());
            Pack.Port = m_UDPPort;
            Pack.Type = IMUPackage::DEVICE_DONGLE;

            uint16_t MsgSize = Pack.toStream(m_Buffer, sizeof(m_Buffer));
            m_Socket.beginPacket(m_BroadcastIP, m_UDPPort); 
            m_Socket.write(m_Buffer, MsgSize);
            m_Socket.endPacket();

            m_LastSearch = millis();

            Serial.printf("Searching for shin trackers ... \n");
        }//if[search for shin trackers]

        // received package?
        while(0 != m_Socket.parsePacket()){
            uint16_t MsgSize = m_Socket.read(m_Buffer, sizeof(m_Buffer));
            if(MsgSize == 0) continue;

            IMUPackage Pack;
            Pack.fromStream(m_Buffer, sizeof(m_Buffer));

            switch(Pack.Cmd){
                case IMUPackage::CMD_SEARCH:{
                    // respond to search
                    if(Pack.Type == IMUPackage::DEVICE_TRACKER_LEFT){
                        m_ShinTrackersIP[LEFT].fromString(Pack.pIP);
                        m_ShinTrackersPort[LEFT] = Pack.Port;
                        Serial.printf("Shin tracker left responded!\n");
                        Serial.printf("\t IP: %s::%d\n", m_ShinTrackersIP[LEFT].toString().c_str(), m_ShinTrackersPort[LEFT]);

                        Pack.Port = m_ShinTrackersPort[LEFT];
                        Pack.setIP(m_ShinTrackersIP[LEFT].toString().c_str());
                        Pack.Cmd = IMUPackage::CMD_CALIBRATE;
                        uint16_t MsgSize = Pack.toStream(m_Buffer, sizeof(m_Buffer));
                        m_Socket.beginPacket(m_ShinTrackersIP[LEFT].toString().c_str(), m_ShinTrackersPort[LEFT]);
                        m_Socket.write(m_Buffer, MsgSize);
                        m_Socket.endPacket();

                    }else if(Pack.Type == IMUPackage::DEVICE_TRACKER_RIGHT){
                        m_ShinTrackersIP[RIGHT].fromString(Pack.pIP);
                        m_ShinTrackersPort[RIGHT] = Pack.Port;
                        Serial.printf("Shin tracker right responded!\n");
                        Serial.printf("\t IP: %s::%d\n", m_ShinTrackersIP[RIGHT].toString().c_str(), m_ShinTrackersPort[RIGHT]);

                        Pack.Port = m_ShinTrackersPort[RIGHT];
                        Pack.setIP(m_ShinTrackersIP[RIGHT].toString().c_str());
                        Pack.Cmd = IMUPackage::CMD_CALIBRATE;
                        uint16_t MsgSize = Pack.toStream(m_Buffer, sizeof(m_Buffer));
                        m_Socket.beginPacket(m_ShinTrackersIP[RIGHT].toString().c_str(), m_ShinTrackersPort[RIGHT]);
                        m_Socket.write(m_Buffer, MsgSize);
                        m_Socket.endPacket();
                    }
                };
                case IMUPackage::CMD_AVG_DATA:
                case IMUPackage::CMD_RAW_DATA:{
                    // update data

                    if(Pack.Type == IMUPackage::DEVICE_TRACKER_LEFT){
                        m_DataLeft = Pack;
                    }else if(Pack.Type == IMUPackage::DEVICE_TRACKER_RIGHT){
                        m_DataRight = Pack;
                    }
                }break;
                default: break;
            }//switch[command]

        }//while[received package]

         if(m_ShinTrackersPort[0] != 0 && m_ShinTrackersPort[1] != 0 && millis() - m_LastPrint > 10){
            printCurrentToSerial();
            m_LastPrint = millis();
        }//printToSerial

    }//end

    void WIPAdapter::printCurrentToSerial(void){
        const char *pStartTag = "##S##";
        const char *pEndTag = "##E##";
        // print accel and gyro data enclosed in start and end tag
        Serial.printf("%s%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%s", pStartTag, 
        m_DataLeft.AccelX, m_DataLeft.AccelY, m_DataLeft.AccelZ, m_DataLeft.GyroX, m_DataLeft.GyroY, m_DataLeft.GyroZ, 
        m_DataRight.AccelX, m_DataRight.AccelY, m_DataRight.AccelZ, m_DataRight.GyroX, m_DataRight.GyroY, m_DataRight.GyroZ, 
        pEndTag);
    }//printCurrentToSerial

}//name space