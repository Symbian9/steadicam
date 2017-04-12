/**
* @file mpu9255.c
* @author Mikhail Komakhin
* @brief Function implementation to communicate with MPU9255 accelerometr
*/

#include "mpu9255.h"

/** 
* Get register value
* @param MPU9255	pointer to MPU9255_t structure, that contains information about MPU9255 connected
* @param address  pointer to register address to read
* @param data		  pointer to output data
* @return				  MPU9255 status
*/
MPU9255_Result_t MPU9255_GetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data){
	uint8_t byteSend = *address;
	uint8_t SPIData;
	// set first bit in '1' that points on read action
	byteSend|=(1<<7);
	/* === Data exchange cycle start === */
	// set CS to 'low' state
	HAL_GPIO_WritePin(MPU9255->SPI_CS_PORT, MPU9255->SPI_CS_PIN, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){ 
		// start error function handler
		return MPU9255_Result_Error;
	} 
	// wait until SPI is busy
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {}
	// dummy byte to send
	byteSend = 0x00;
	// transmit dummy byte and recieve register value at the same time
	if (HAL_SPI_TransmitReceive_DMA(MPU9255->SPI, &byteSend, &SPIData, sizeof(SPIData)) != HAL_OK){ 
		// start error function handler
		return MPU9255_Result_Error;
	}
	// wait until SPI is busy
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX_RX) {}
	// set CS to 'high' state
	HAL_GPIO_WritePin(MPU9255->SPI_CS_PORT, MPU9255->SPI_CS_PIN, GPIO_PIN_SET);
	/* === Data exchange cycle end === */
		
	*data = SPIData;
	return MPU9255_Result_Ok;
};

/** 
* Set register value
* @param MPU9255	pointer to MPU9255_t structure, that contains information about MPU9255 connected
* @param address  pointer to register address to write
* @param data		  pointer to register value to write
* @return				  MPU9255 status
*/
MPU9255_Result_t MPU9255_SetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data){
	uint8_t byteSend = *address;
	/* === Data exchange cycle start === */
	// set CS to 'low' state
	HAL_GPIO_WritePin(MPU9255->SPI_CS_PORT, MPU9255->SPI_CS_PIN, GPIO_PIN_RESET);
	// transmit register address
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){
		// start error function handler
		return MPU9255_Result_Error;
	}
	// wait until SPI is busy
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {}
	byteSend = *data;
	// transmit register value
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){
		// start error function handler
		return MPU9255_Result_Error;
	}
	// wait until SPI is busy
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {}
	// set CS to 'high' state
	HAL_GPIO_WritePin(MPU9255->SPI_CS_PORT, MPU9255->SPI_CS_PIN, GPIO_PIN_SET);
	/* === Data exchange cycle end === */
		
	return MPU9255_Result_Ok;
};

/** 
* Get multiple registers value
* @param MPU9255	pointer to MPU9255_t structure, that contains information about MPU9255 connected
* @param address  pointer to register start address to read
* @param data		  pointer to output data
* @param number	  pointer to number of bytes to read
* @return				  MPU9255 status
*/
MPU9255_Result_t MPU9255_GetMultiReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t data[], uint8_t number){
	uint8_t byteSend = *address;
	//uint8_t* SPIData_1 = malloc(number * sizeof(uint8_t));
	uint8_t i;
	// set first bit in '1' that points on read action
	byteSend|=(1<<7);
	/* === Data exchange cycle start === */
	// set CS to 'low' state
	HAL_GPIO_WritePin(MPU9255->SPI_CS_PORT, MPU9255->SPI_CS_PIN, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){ 
		// start error function handler
		return MPU9255_Result_Error;
	} 
	// wait until SPI is busy
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {}
	// dummy byte to send
	byteSend = 0x00;
	for (i=0; i<number; i++) {
		// transmit dummy byte and recieve register value at the same time
		if (HAL_SPI_TransmitReceive_DMA(MPU9255->SPI, &byteSend, &data[i], sizeof(data[i])) != HAL_OK){ 
			// start error function handler
			return MPU9255_Result_Error;
		}
		// wait until SPI is busy
		while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX_RX) {}
	}
	// set CS to 'high' state
	HAL_GPIO_WritePin(MPU9255->SPI_CS_PORT, MPU9255->SPI_CS_PIN, GPIO_PIN_SET);
	/* === Data exchange cycle end === */

	return MPU9255_Result_Ok;
};

/** 
* Initialize accelerometr with parameters
* @param MPU9255	pointer to MPU9255_t structure, that contains information about MPU9255 connected
* @return				  MPU9255 status
*/
MPU9255_Result_t MPU9255_Init(MPU9255_t* MPU9255) {
	
}

/** 
* Read accelerometr values on all axis
* @param MPU9255	pointer to MPU9255_t structure, that contains information about MPU9255 connected
* @return				  MPU9255 status
*/
MPU9255_Result_t MPU9255_ReadAccel(MPU9255_t* MPU9255) {
	uint8_t data[6]={0};
    
	/* Read accelerometer data */
	uint8_t address = MPU9255_ACCEL_XOUT_H;
	MPU9255_GetMultiReg(MPU9255, &address, data, 6);
	
	MPU9255->Ax_Raw = ((int16_t)data[0] << 8) | data[1];
	MPU9255->Ay_Raw = ((int16_t)data[2] << 8) | data[3];  
	MPU9255->Az_Raw = ((int16_t)data[4] << 8) | data[5];
	
	MPU9255->Ax = (float)MPU9255->Ax_Raw * MPU9255->AMult;
	MPU9255->Ay = (float)MPU9255->Ay_Raw * MPU9255->AMult;
	MPU9255->Az = (float)MPU9255->Az_Raw * MPU9255->AMult;
	
	return MPU9255_Result_Ok;
}

/** 
* Read gyroscope values on all axis
* @param MPU9255	pointer to MPU9255_t structure, that contains information about MPU9255 connected
* @return				  MPU9255 status
*/
MPU9255_Result_t MPU9255_ReadGyro(MPU9255_t* MPU9255) {
	uint8_t data[6]={0};
	
	/* Read gyroscope data */
	uint8_t address = MPU9255_GYRO_XOUT_H;
	MPU9255_GetMultiReg(MPU9255, &address, data, 6);
	
	MPU9255->Gx_Raw = ((int16_t)data[0] << 8) | data[1];
	MPU9255->Gy_Raw = ((int16_t)data[2] << 8) | data[3];  
	MPU9255->Gz_Raw = ((int16_t)data[4] << 8) | data[5];
	
	MPU9255->Gx = (float)MPU9255->Gx_Raw * MPU9255->GMult;
	MPU9255->Gy = (float)MPU9255->Gy_Raw * MPU9255->GMult;
	MPU9255->Gz = (float)MPU9255->Gz_Raw * MPU9255->GMult;
	
	return MPU9255_Result_Ok;
}
