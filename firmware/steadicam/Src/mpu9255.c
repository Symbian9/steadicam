/**
* @file mpu9255.c
* @author Mikhail Komakhin
* @brief Function implementation to communicate with MPU9255 accelerometr
*/

#include "mpu9255.h"
#include "spi.h"

MPU9255_Result_t MPU9255_GetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data){
	
	uint8_t byteSend = *address;
	uint8_t SPIData;
	byteSend|=(1<<7); // set first bit in '1' that points on read action
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){ 
		// start error function handler
		return MPU9255_Result_Error;
	} 
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {} // wait until SPI is busy
		
	byteSend = 0x00; // dummy byte send
	if (HAL_SPI_TransmitReceive_DMA(MPU9255->SPI, &byteSend, &SPIData, sizeof(SPIData)) != HAL_OK){ // transmit dummy byte and recieve register value at the same time
		// start error function handler
		return MPU9255_Result_Error;
	}
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX_RX) {} // wait until SPI is busy
	*data = SPIData;
	return MPU9255_Result_Ok;
};

MPU9255_Result_t MPU9255_SetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data){
	uint8_t byteSend = *address;
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){ // transmit register address
		// start error function handler
		return MPU9255_Result_Error;
	}
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {} // wait until SPI is busy
		
	byteSend = *data;
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){ // transmit register value
		// start error function handler
		return MPU9255_Result_Error;
	}
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {} // wait until SPI is busy
	return MPU9255_Result_Ok;
};

MPU9255_Result_t MPU9255_ReadAccel(MPU9255_t* MPU9255) {
    uint8_t data[6];
    
    /* Read accelerometer data */
		uint8_t address;
		CS_SPI3_LOW();
		address=MPU9255_ACCEL_XOUT_H; 
		MPU9255_GetReg(MPU9255, &address, &data[0]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_ACCEL_XOUT_L; 
		MPU9255_GetReg(MPU9255, &address, &data[1]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_ACCEL_YOUT_H; 
		MPU9255_GetReg(MPU9255, &address, &data[2]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_ACCEL_YOUT_L; 
		MPU9255_GetReg(MPU9255, &address, &data[3]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_ACCEL_ZOUT_H; 
		MPU9255_GetReg(MPU9255, &address, &data[4]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_ACCEL_ZOUT_L; 
		MPU9255_GetReg(MPU9255, &address, &data[5]);
		CS_SPI3_HIGH();
	
    MPU9255->Ax_Raw = ((int16_t)data[0] << 8) | data[1];
    MPU9255->Ay_Raw = ((int16_t)data[2] << 8) | data[3];  
    MPU9255->Az_Raw = ((int16_t)data[4] << 8) | data[5];
    
    MPU9255->Ax = (float)MPU9255->Ax_Raw * MPU9255->AMult;
    MPU9255->Ay = (float)MPU9255->Ay_Raw * MPU9255->AMult;
    MPU9255->Az = (float)MPU9255->Az_Raw * MPU9255->AMult;
		
		return MPU9255_Result_Ok;
}

MPU9255_Result_t MPU9255_ReadGyro(MPU9255_t* MPU9255) {
    uint8_t data[6];
    
		uint8_t address;
		CS_SPI3_LOW();
		address=MPU9255_GYRO_XOUT_H; 
		MPU9255_GetReg(MPU9255, &address, &data[0]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_GYRO_XOUT_L; 
		MPU9255_GetReg(MPU9255, &address, &data[1]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_GYRO_YOUT_H; 
		MPU9255_GetReg(MPU9255, &address, &data[2]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_GYRO_YOUT_L; 
		MPU9255_GetReg(MPU9255, &address, &data[3]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_GYRO_ZOUT_H; 
		MPU9255_GetReg(MPU9255, &address, &data[4]);
		CS_SPI3_HIGH();
		HAL_Delay(5);
		CS_SPI3_LOW();
		address=MPU9255_GYRO_ZOUT_L; 
		MPU9255_GetReg(MPU9255, &address, &data[5]);
		CS_SPI3_HIGH();
    
    MPU9255->Gx_Raw = ((int16_t)data[0] << 8) | data[1];
    MPU9255->Gy_Raw = ((int16_t)data[2] << 8) | data[3];  
    MPU9255->Gz_Raw = ((int16_t)data[4] << 8) | data[5];
    
    MPU9255->Gx = (float)MPU9255->Gx_Raw * MPU9255->GMult;
    MPU9255->Gy = (float)MPU9255->Gy_Raw * MPU9255->GMult;
    MPU9255->Gz = (float)MPU9255->Gz_Raw * MPU9255->GMult;
		
		return MPU9255_Result_Ok;
}
