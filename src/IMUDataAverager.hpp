/*****************************************************************************\
*                                                                           *
* File(s): IMUDataAverager.h and IMUDataAverager.cpp                         *
*                                                                           *
* Content: Class to average IMU Data   *
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
#ifndef __IMUWIP_IMUDATAAVERAGER_HPP__
#define __IMUWIP_IMUDATAAVERAGER_HPP__

#include <inttypes.h>
#include <MPU6050.h>

namespace IMUWIP{
    class IMUDataAverager{
        public:
        IMUDataAverager(void){
            m_BufferSize = 0;
            m_pBuffer = nullptr;
            m_AvgTime = 0;
        }//constructor

        ~IMUDataAverager(void){
            end();
        }//Destructor

        void begin(uint16_t MaxBuffer, uint32_t AveragingTime){
            end();
            m_BufferSize = MaxBuffer;
            m_pBuffer = new BufferItem[m_BufferSize];
            m_AvgTime = AveragingTime;
        }//begin

        void end(void){
            if(nullptr != m_pBuffer) delete[] m_pBuffer;
            m_pBuffer = nullptr;
            m_BufferSize = 0;
            m_AvgTime = 0;
        }//end

        void addData(ArduForge::MPU6050::SensorData SData){
            // search for empty space and write data
            for(uint32_t i=0; i < m_BufferSize; ++i){
                if(millis() - m_pBuffer[i].Timestamp > m_AvgTime){
                    m_pBuffer[i].SData = SData;
                    m_pBuffer[i].Timestamp = millis();
                    return;
                }
            }
        }//addData

        void retrieveData(ArduForge::MPU6050::SensorData *pAvgData){
            float Scale = 0.0f;
            pAvgData->AccelX = 0.0f;
            pAvgData->AccelY = 0.0f;
            pAvgData->AccelZ = 0.0f;
            pAvgData->GyroX = 0.0f;
            pAvgData->GyroY = 0.0f;
            pAvgData->GyroZ = 0.0f;

            const uint32_t Stamp = millis();
            for(uint32_t i=0; i < m_BufferSize; ++i){
                if(Stamp - m_pBuffer[i].Timestamp <= m_AvgTime){
                    pAvgData->AccelX += m_pBuffer[i].SData.AccelX;
                    pAvgData->AccelY += m_pBuffer[i].SData.AccelY;
                    pAvgData->AccelZ += m_pBuffer[i].SData.AccelZ;

                    pAvgData->GyroX += m_pBuffer[i].SData.GyroX;
                    pAvgData->GyroY += std::abs(m_pBuffer[i].SData.GyroY);
                    pAvgData->GyroZ += m_pBuffer[i].SData.GyroZ;

                    Scale += 1.0f;
                }
            }//for[buffer]

            if(Scale > 1.0f){
                pAvgData->AccelX /= Scale;
                pAvgData->AccelY /= Scale;
                pAvgData->AccelZ /= Scale;

                pAvgData->GyroX /= Scale;
                pAvgData->GyroY /= Scale;
                pAvgData->GyroZ /= Scale;
            }

        }//retrieveData

        protected:
        struct BufferItem{
            ArduForge::MPU6050::SensorData SData;
            uint32_t Timestamp;
        };//BufferItem
        
        uint16_t m_BufferSize;
        BufferItem *m_pBuffer;  
        uint32_t m_AvgTime;

    };//IMUDataAverager

}

#endif 