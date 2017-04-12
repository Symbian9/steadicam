/**
* @file mpu9255.h
* @author Mikhail Komakhin
* @brief Function prototypes and defines to communicate with MPU9255 accelerometr
*/

#ifndef MPU9255_H
#define MPU9255_H

#include "stm32f4xx.h"

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
#define MPU9255_ACCEL_CONF_1					 28

typedef enum _MPU9255_AccelSens_t {
    MPU9255_AcceSens_2G = 0x00,
    MPU9255_AcceSens_4G = 0x01,
    MPU9255_AcceSens_8G = 0x02,
    MPU9255_AcceSens_16G = 0x03
} MPU9255_AccelSens_t;

typedef enum _MPU9255_GyroSens_t {
    MPU9255_GyroSens_250DPS = 0x00,
    MPU9255_GyroSens_500DPS = 0x01,
    MPU9255_GyroSens_1000DPS = 0x02,
    MPU9255_GyroSens_2000DPS = 0x03
} MPU9255_GyroSens_t;

typedef enum _MPU9255_MagSens_t {
    MPU9255_MagSens_14Bit = 0x00,    // 0.6 mG per LSB
    MPU9255_MagSens_16Bit            // 0.15 mG per LSB
} MPU9255_MagSens_t;

typedef struct _MPU9255_InitTypeDef {
		MPU9255_AccelSens_t ASense;
		MPU9255_GyroSens_t GSense;
		MPU9255_MagSens_t MSense;
} MPU9255_InitTypeDef;

typedef enum _MPU9255_Result_t{
    MPU9255_Result_Ok = 0x00,
    MPU9255_Result_Error,
    MPU9255_Result_DeviceNotConnected
} MPU9255_Result_t;

typedef struct _MPU9255_t {
		SPI_HandleTypeDef* SPI;
		GPIO_TypeDef* SPI_CS_PORT;
		uint16_t SPI_CS_PIN;
		MPU9255_InitTypeDef Init;
    float Ax, Ay, Az;         /*!< Accelerometer data */
    float Gx, Gy, Gz;         /*!< Gyroscope data */
    float Mx, My, Mz;         /*!< Magnetometer data */
	
	  int16_t Ax_Raw, Ay_Raw, Az_Raw;         /*!< Accelerometer raw data */
    int16_t Gx_Raw, Gy_Raw, Gz_Raw;         /*!< Gyroscope raw data */
    int16_t Mx_Raw, My_Raw, Mz_Raw;         /*!< Magnetometer raw data */
    
    float AMult, GMult, MMult;
	
} MPU9255_t;

MPU9255_Result_t MPU9255_GetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data);
MPU9255_Result_t MPU9255_SetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data);


MPU9255_Result_t MPU9255_Init(MPU9255_t* MPU9255);

MPU9255_Result_t MPU9255_ReadAccel(MPU9255_t* MPU9255);
MPU9255_Result_t MPU9255_ReadGyro(MPU9255_t* MPU9255);
MPU9255_Result_t MPU9255_ReadMag(MPU9255_t* MPU9255);
MPU9255_Result_t MPU9255_DataReady(MPU9255_t* MPU9255);

#endif
