#include "ShinTracker.h"

namespace IMUWIP{
    ShinTracker::ShinTracker(void){
        m_pWire = nullptr;
        m_LastIMURead = 0;
        m_LastPackage = 0;

        m_TargetPort = 0;
    }//Constructor

    ShinTracker::~ShinTracker(void){

    }//Destructor

    void ShinTracker::begin(TwoWire *pWire, IMUPackage::DeviceType Type, const uint16_t Port){
        m_StartTime = millis();

        m_pWire = pWire;
        m_Socket.begin(Port);
        
        m_IMU.begin();
        m_IMU.calibrate();

        m_DataAvg.begin(30, 200);

        m_TargetPort = 0;

        m_Type = Type;
    }//begin

    void ShinTracker::end(void){

    }//end

    void ShinTracker::update(void){
        if(millis() - m_LastIMURead > m_IMUSamplingIntervall){
            // read IMU Data
            m_DataAvg.addData(m_IMU.read());
            m_LastIMURead = millis();
        }

        if(m_TargetPort > 0 && millis() - m_LastPackage > m_PackageSendingIntervall){
            // read IMU data
            ArduForge::MPU6050::SensorData Data;
            m_DataAvg.retrieveData(&Data);
            IMUPackage Package;
            Package.GyroX = Data.GyroX;
            Package.GyroY = Data.GyroY;
            Package.GyroZ = Data.GyroZ;
            Package.AccelX = Data.AccelX;
            Package.AccelY = Data.AccelY;
            Package.AccelZ = Data.AccelZ;
            Package.Type = m_Type;
            Package.Cmd = IMUPackage::CMD_AVG_DATA;
            
            // pack to stream and send
            uint16_t MsgSize = Package.toStream(m_Buffer, sizeof(m_Buffer));
            m_Socket.beginPacket(m_TargetIP, m_TargetPort);
            m_Socket.write(m_Buffer, MsgSize);
            m_Socket.endPacket();

            m_LastPackage = millis();
        }

        // Did we receive data?
        m_Socket.parsePacket();
        int16_t MsgSize = m_Socket.read(m_Buffer, sizeof(m_Buffer));

        if(MsgSize > 0){
            if(IMUPackage::checkMagicTag(m_Buffer)){
                IMUPackage P;
                P.fromStream(m_Buffer, MsgSize);

                if(P.Cmd == IMUPackage::CMD_CALIBRATE){
                    m_IMU.calibrate();
                }else if(P.Cmd == IMUPackage::CMD_SEARCH){
                    // we have a target for our data
                    m_TargetIP.fromString(P.pIP);
                    m_TargetPort = P.Port;

                    Serial.printf("Connected to Host: %s::%d\n", m_TargetIP.toString().c_str(), m_TargetPort);

                    // send acknowledge
                    P.setIP(WiFi.localIP().toString().c_str());
                    P.Port = m_Socket.localPort();
                    P.Type = m_Type;

                    uint16_t MsgSize = P.toStream(m_Buffer, sizeof(m_Buffer));
                    m_Socket.beginPacket(m_TargetIP, m_TargetPort);
                    m_Socket.write(m_Buffer, MsgSize);
                    m_Socket.endPacket();
                }
            }//if[IMUPackage]

        }//if[received message]
    }//update

}//name-space