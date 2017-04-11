/**
* @file mpu9255.c
* @author Mikhail Komakhin
* @brief Function implementation to communicate with MPU9255 accelerometr
*/

#include "mpu9255.h"
/** 
* Get register value
* @param hspi		  SPI interface handle structure
* @param address  Register address to read
* @return				  Register value
*/
int8_t MPU9255_GetReg(MPU9255_t* MPU9255, uint8_t *address){
	
	uint8_t byteSend = *address;
	uint8_t SPIData;
	byteSend|=(1<<7); // set first bit in '1' that points on read action
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){ 
		// start error function handler
	} 
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {} // wait until SPI is busy
	byteSend = 0x00; // dummy byte send
	if (HAL_SPI_TransmitReceive_DMA(MPU9255->SPI, &byteSend, &SPIData, sizeof(SPIData)) != HAL_OK){ // transmit dummy byte and recieve register value at the same time
		// start error function handler
	}
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX_RX) {} 
	return SPIData;
};

/** 
* Write register value
* @param hspi		  SPI interface handle structure
* @param address  Register address to write
* @param data		  Register value to write
*/
void MPU9255_SetReg(MPU9255_t* MPU9255, uint8_t *address, uint8_t *data){
	uint8_t byteSend = *address;
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){
		// start error function handler
	} // transmit register address
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {} // wait until SPI is busy
	byteSend = *data;
	if (HAL_SPI_Transmit_DMA(MPU9255->SPI, &byteSend, sizeof(byteSend)) != HAL_OK){
		// start error function handler
	} // transmit register value
	while (HAL_SPI_GetState(MPU9255->SPI) == HAL_SPI_STATE_BUSY_TX) {}		
};

/** 
* Init MPU9255 with following parametrs
* @param hspi		  SPI interface handle structure
* @param hmpu9255 MPU9255 configuration structure
*/
//void MPU9255_Init(SPI_HandleTypeDef *hspi, MPU9255_InitTypeDef *hmpu9255){

//}



MPU9255_Result_t MPU9255_ReadAccel(MPU9255_t* MPU9255) {
    uint8_t data[6];
    
    /* Read accelerometer data */
    // TM_I2C_ReadMulti(MPU9250_I2C, MPU9250->I2C_Addr, ACCEL_XOUT_H, data, 6);
    
    MPU9255->Ax_Raw = ((int16_t)data[0] << 8) | data[1];
    MPU9255->Ay_Raw = ((int16_t)data[2] << 8) | data[3];  
    MPU9255->Az_Raw = ((int16_t)data[4] << 8) | data[5];
    
    MPU9255->Ax = (float)MPU9255->Ax_Raw * MPU9255->AMult;
    MPU9255->Ay = (float)MPU9255->Ay_Raw * MPU9255->AMult;
    MPU9255->Az = (float)MPU9255->Az_Raw * MPU9255->AMult;
}

MPU9255_Result_t MPU9255_ReadGyro(MPU9255_t* MPU9255) {
    uint8_t data[6];
    //TM_I2C_ReadMulti(MPU9250_I2C, MPU9250->I2C_Addr, GYRO_XOUT_H, data, 6);
    
    MPU9255->Gx_Raw = ((int16_t)data[0] << 8) | data[1];
    MPU9255->Gy_Raw = ((int16_t)data[2] << 8) | data[3];  
    MPU9255->Gz_Raw = ((int16_t)data[4] << 8) | data[5];
    
    MPU9255->Gx = (float)MPU9255->Gx_Raw * MPU9255->GMult;
    MPU9255->Gy = (float)MPU9255->Gy_Raw * MPU9255->GMult;
    MPU9255->Gz = (float)MPU9255->Gz_Raw * MPU9255->GMult;
}
