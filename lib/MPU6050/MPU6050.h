/*****************************************************************************\
*                                                                           *
* File(s): MPU6050.h and MPU6050.cpp                                        *
*                                                                           *
* Content: Class to communicate with an MPU6050 inertial measuring unit.    *
*                                                                           *
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
#ifndef ARDUFORGE_MPU6050_H
#define ARDUFORGE_MPU6050_H

#include <inttypes.h>
#include <Wire.h>

namespace ArduForge{
    /**
     * \brief Class that handles communication with an MPU6050 inertial measurement unit chip. It also converts the raw data to interpretable values.
     * 
     * The MPU6050 is an 6-Axis accelerometer and gyroscope (3-axis for acceleration and 3 for rotation), hence you get the acceleration (actually the orientation) and rotation values in x,y, and z direction.
     * \image html MPU6050Wiring.png
     */
    class MPU6050{
    public:
   
        /**
         *  \brief Struct to store the raw sensor values.
         */
        struct RawSensorData{
            int16_t AccelX; ///< Acceleration x axis
            int16_t AccelY; ///< Acceleration y axis
            int16_t AccelZ; ///< Acceleration z axis
            int16_t GyroX; ///< Rotation x axis
            int16_t GyroY; ///< Rotation y axis
            int16_t GyroZ; ///< Rotation z axis
            int16_t Temperature; ///< Raw temperature reading

            RawSensorData(void){
                AccelX = AccelY = AccelZ = 0;
                GyroX = GyroY = GyroZ = 0;
                Temperature = 0.0;
            }//Constructor

        };//RawSensorData

        /**
         * \brief Struct to store interpreted sensor values.
         */
        struct SensorData{
            float AccelX; ///< Acceleration x axis
            float AccelY; ///< Acceleration y axis
            float AccelZ; ///< Acceleration z axis
            float GyroX; ///< Rotation x axis
            float GyroY; ///< Rotation y axis
            float GyroZ; ///< Rotation z axis
            float Temperature; ///< Temperature (of chip, not room temperature)
        };//SensorData

        /**
         * \brief Gyroscope sensitivity configuration values. Values are in \f$ \pm \f$ degree per second
         */
        enum struct GyroConfig : int8_t{
            SCALE_250 = 0, ///< Scale 250
            SCALE_500, ///< Scale 500
            SCALE_1000, ///< Scale 1000
            SCALE_2000, ///< Scale 2000
        };//GyroConfig

        /**
         * \brief Accelerometer sensitivity configuration values. Values is LSB per g.
         */
        enum struct AccelConfig: int8_t{
            SCALE_2G = 0, ///< 2g sensitivity
            SCALE_4G, ///< 4g sensitivity
            SCALE_8G, ///< 8g sensitivity
            SCALE_16G, ///< 16g sensitivity
        };//AccelConfig

        /**
         * \brief Constructor
         */
        MPU6050(void);

        /**
         * \brief Destructor
         */
        ~MPU6050(void);

        /**
         *  \brief Initializes communication with the MPU6050 device.
         *  
         * \param[in] pWire I2C interface. By default it uses the default I2C that is defined by Arduino.
         * \param[in] Addr I2C address of the device. Usually 0x68 or 0x69 if AD0 is HIGH.
         * \param[in] AC Accelerometer sensitivity. 
         * \param[in] GC Gyroscope sensitivity.
         */
        void begin(TwoWire *pWire = &Wire, uint8_t Addr = 0x68, AccelConfig AC = AccelConfig::SCALE_4G, GyroConfig GC = GyroConfig::SCALE_500);

        /**
         * \brief Ends communication with the device.
         */
        void end(void);

        /**
         * \brief Puts device in sleep mode. Reduced energy consumption, no sensor data reading possible.
         */
        void sleep(void);

        /**
         * \brief Wakes up the device. Reading sensor values possible again.
         */
        void wake(void);

        /**
         * \brief Sets accelerometer sensitivity.
         * 
         * \param[in] Value Configuration value
         */
        void configAccel(AccelConfig Value);

        /**
         * \brief Sets gyroscope sensitivity value.
         * 
         * \param[in] Value Configuration value
         */
        void configGyro(GyroConfig Value);

        /**
         * \brief Enables/disables MPU6050's internal thermometer.
         * 
         * \param[in] Enable True to enable and false to disable thermometer.
         */
        void configThermistor(bool Enable);

        /**
         * \brief Returns raw sensor reading.
         * 
         * \return Raw sensor readings.
         */
        RawSensorData readRaw(void)const;

        /**
         * \brief Reads the sensor data and converts it to interpretable values.
         * 
         * \return Interpreted sensor data values.
         */
        SensorData read(void)const;

        /**
         * \brief Recalibrates the device.
         * 
         * This method will read and store the current data registers and all following readings will the relative to most recent calibration data.
         */
        void calibrate(void);
    protected:
        /**
         * \brief Returns accelerometer sensitivity scaling value. Required to interpret raw sensor readings.
         * 
         * \param[in] Value Configuration value
         * \return Accelerometer sensitivity scaling value.
         */
        int16_t accelSensivity(AccelConfig Value)const;

        /**
         * \brief Returns gyroscope sensitivity scaling value. Required to interpret raw sensor readings.
         * 
         * \param[in] Value Configuration value
         * \return Gyroscope sensitivity scaling value.
         */
        float gyroSensivity(GyroConfig Value)const;

        /**
         * \brief Reads a register via I2C
         * 
         * \param[in] Reg Register address
         * \return Register value
         */
        uint8_t readRegister(uint8_t Reg)const;

        uint8_t m_SensorAddr; ///< MPU6050's I2C address
        TwoWire *m_pWire; ///< I2C interface
        AccelConfig m_AccelConfig; ///< Current accelerometer sensitivity configuration
        GyroConfig m_GyroConfig; ///< Current gyroscope sensitivity configuration
        RawSensorData m_Calibration; ///< Calibration data
    };//MPU6050

}//names-pace


#endif