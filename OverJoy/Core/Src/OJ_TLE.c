/*
 * OJ_TLE.c
 *
 *  Created on: 16 ???. 2019 ?.
 *      Author: DIIV
 */

#include "OJ_TLE.h"

extern SPI_HandleTypeDef hspi2;

uint8_t TLEReadData(uint8_t dataType, void *pData)
{
	dataStructTLETypeDef *dataStructTLE = (dataStructTLETypeDef *)malloc(sizeof(dataStructTLETypeDef));
	uint8_t bufRX[16];
	HAL_StatusTypeDef ret;
	uint16_t bufTX;
	uint8_t temp;

	switch (dataType)
	{
		case TLE_MAIN_VALUES:
			{
				bufTX = TLE_UPDATE;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_I;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
			 	ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)bufRX, 7);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_II;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
				ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)bufRX+7, 7);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_III;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
				ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)bufRX+14, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//				temp = dataStructTLE->valueX >> 8;
//				dataStructTLE->valueX = (dataStructTLE->valueX << 8) + temp;
//				temp = dataStructTLE->valueY >> 8;
//				dataStructTLE->valueY = (dataStructTLE->valueY << 8) + temp;

				memcpy(pData, &dataStructTLE->valueX, 4);
			}
			break;
		case TLE_TEMP_VALUE:
			{
				bufTX = 0x715A;
				temp = bufTX >> 8;
				bufTX = (bufTX << 8) + temp;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = 0x6180;
				temp = bufTX >> 8;
				bufTX = (bufTX << 8) + temp;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_UPDATE;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_I;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
			 	ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)bufRX, 7);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_II;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
				ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)bufRX+7, 7);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_III;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
				ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)bufRX+14, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = 0x6100;
				temp = bufTX >> 8;
				bufTX = (bufTX << 8) + temp;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				memcpy(pData, &dataStructTLE->valueX, 2);
			}
			break;

	}

	dataStructTLE->ctrl1	 = bufRX[0];
	dataStructTLE->valueX	 = (bufRX[2] << 8) + bufRX[1];
	dataStructTLE->valueY 	 = (bufRX[4] << 8) + bufRX[3];
	dataStructTLE->fcn_stat	 = bufRX[5];
	dataStructTLE->fsync_inv = bufRX[6];
	dataStructTLE->angt 	 = bufRX[7];
	dataStructTLE->reserved1 = bufRX[8];
	dataStructTLE->reserved2 = bufRX[9];
	dataStructTLE->reserved3 = bufRX[10];
	dataStructTLE->reserved4 = bufRX[11];
	dataStructTLE->tst		 = bufRX[12];
	dataStructTLE->deviceID  = bufRX[13];
	dataStructTLE->lock		 = bufRX[14];
	dataStructTLE->ctrl2	 = bufRX[15];

//	dataStructTLE->valueX = 0xFFFF - dataStructTLE->valueX;
//	dataStructTLE->valueY = 0xFFFF - dataStructTLE->valueY;

	switch (dataType)
	{
		case TLE_MAIN_VALUES:
			{
				memcpy(pData, &dataStructTLE->valueX, 4);
			}
			break;
		case TLE_TEMP_VALUE:
			{
				memcpy(pData, &dataStructTLE->valueX, 4);
			}
			break;

	}

	free(dataStructTLE);
	return 0;
}

uint8_t TLEConfig(uint8_t confType, void *pData)
{
	HAL_StatusTypeDef ret;
	uint16_t bufTX;
	uint8_t temp;

	switch (confType)
	{
		case TLE_CFG_INIT:
			{
				bufTX = 0x715A;
				temp = bufTX >> 8;
				bufTX = (bufTX << 8) + temp;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = 0x0903;
				temp = bufTX >> 8;
				bufTX = (bufTX << 8) + temp;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
			}
			break;
	}
	return 0;
}
