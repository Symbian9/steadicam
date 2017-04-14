/**
* @file mpu9255.h
* @author Mikhail Komakhin
* @brief Function prototypes and defines to communicate with MPU9255 accelerometr
*/

#ifndef MPU9255_H
#define MPU9255_H

#include "stm32f4xx.h"

/* Register addresses defines */
#define MPU9255_ACCEL_XOUT_H           59
#define MPU9255_ACCEL_XOUT_L           60
#define MPU9255_ACCEL_YOUT_H           61
#define MPU9255_ACCEL_YOUT_L           62
#define MPU9255_ACCEL_ZOUT_H           63
#define MPU9255_ACCEL_ZOUT_L           64
#define MPU9255_WHO_AM_I							 117
#define MPU9255_GYRO_XOUT_H						 67
#define MPU9255_GYRO_XOUT_L						 68
#define MPU9255_GYRO_YOUT_H						 69
#define MPU9255_GYRO_YOUT_L						 70
#define MPU9255_GYRO_ZOUT_H						 71
#define MPU9255_GYRO_ZOUT_L						 72
#define MPU9255_GYRO_CONF					 		 27
#define MPU9255_ACCEL_CONF_1					 28

/**
 * @brief Accelerometr scale range enum.
 *
 * MPU9255 sensor accelerometr can function in several scales. You may choose scale using this enum.
 */
typedef enum _MPU9255_AccelSens_t {
    MPU9255_AccelSens_2G = 0x00,		/**< Accelerometr 2G scale range. */
    MPU9255_AccelSens_4G = 0x01,		/**< Accelerometr 4G scale range. */
    MPU9255_AccelSens_8G = 0x02,		/**< Accelerometr 8G scale range. */
    MPU9255_AccelSens_16G = 0x03		/**< Accelerometr 16G scale range. */
} MPU9255_AccelSens_t;

/**
 * @brief Gyroscope scale range enum.
 *
 * MPU9255 sensor agyroscope can function in several scales. You may choose scale using this enum.
 */
typedef enum _MPU9255_GyroSens_t {
    MPU9255_GyroSens_250DPS = 0x00,		/**< Gyroscope 250DPS scale range. */
    MPU9255_GyroSens_500DPS = 0x01,		/**< Gyroscope 500DPS scale range. */
    MPU9255_GyroSens_1000DPS = 0x02,	/**< Gyroscope 1000DPS scale range. */
    MPU9255_GyroSens_2000DPS = 0x03		/**< Gyroscope 200DPS scale range. */
} MPU9255_GyroSens_t;

//TODO:make magnitometr config Doxygen notations
typedef enum _MPU9255_MagSens_t {
    MPU9255_MagSens_14Bit = 0x00,    // 0.6 mG per LSB
    MPU9255_MagSens_16Bit            // 0.15 mG per LSB
} MPU9255_MagSens_t;

/**
 * @brief MPU9255 initialization structure
 *
 * MPU9255 initialization parametrs that determine different sensors scale range.
 */
typedef struct _MPU9255_InitTypeDef {
		MPU9255_AccelSens_t ASense;	/**< Accelerometr scale range. */
		MPU9255_GyroSens_t GSense;	/**< Gyroscope scale range. */
		MPU9255_MagSens_t MSense;		/**< Magnetometr scale range. */
} MPU9255_InitTypeDef;

/**
 * @brief MPU9255 result status structure
 *
 * Various status that may occure during the operating with a sensor.
 */
typedef enum _MPU9255_Result_t{
    MPU9255_Result_Ok = 0x00,					/**< Succcess status. */
    MPU9255_Result_Error,							/**< Error occured. */
    MPU9255_Result_DeviceNotConnected /**< There is no device found. */
} MPU9255_Result_t;

/**
 * @brief MPU9255 handle structure
 *
 * Structure contains pointers to several configs, data recieved from the sensor, raw data and data multiplicators 
 */
typedef struct _MPU9255_t {
		SPI_HandleTypeDef* SPI; 		/**< Pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module that seerves MPU9255. */
		GPIO_TypeDef* SPI_CS_PORT;	/**< CS port that serves MPU9255 connected SPI module. This parametr can be one of GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.*/
		uint16_t SPI_CS_PIN;				/**< CS pin that serves MPU9255 connected SPI module as CS line. This parameter can be one of GPIO_PIN_x where x can be (0..15). */
		MPU9255_InitTypeDef Init; 	/**< Structure that specifies MPU9255 initialization parametrs. */
		
    float Ax, Ay, Az;         	/**< Accelerometer data. */
    float Gx, Gy, Gz;         	/**< Gyroscope data. */
    float Mx, My, Mz;         	/**< Magnetometer data. */
	
	  int16_t Ax_Raw, Ay_Raw, Az_Raw; 		/**< Accelerometer raw data. */
    int16_t Gx_Raw, Gy_Raw, Gz_Raw; 		/**< Gyroscope raw data. */
    int16_t Mx_Raw, My_Raw, Mz_Raw; 		/**< Magnetometer raw data. */
    
    float AMult, GMult, MMult;	/**< Data multiplicators that depend on sensor scale range. */
	
} MPU9255_t;

MPU9255_Result_t MPU9255_GetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data);
MPU9255_Result_t MPU9255_SetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data);


MPU9255_Result_t MPU9255_Init(MPU9255_t* MPU9255);

MPU9255_Result_t MPU9255_ReadAccel(MPU9255_t* MPU9255);
MPU9255_Result_t MPU9255_ReadGyro(MPU9255_t* MPU9255);
MPU9255_Result_t MPU9255_ReadMag(MPU9255_t* MPU9255);
MPU9255_Result_t MPU9255_DataReady(MPU9255_t* MPU9255);

#endif
