/*
 * OJ_TLE.c
 *
 *  Created on: 16 ???. 2019 ?.
 *      Author: DIIV
 */

#include "OJ_TLE.h"

extern SPI_HandleTypeDef hspi2;

extern mainDataTypeDef mainData;

uint8_t TLEReadData(uint8_t dataType, void *pData)
{
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
			 	ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)&mainData.allReg, 7);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_II;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
				ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)&mainData.allReg+7, 7);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_III;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
				ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)&mainData.allReg+14, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				mainData.currentX = (mainData.allReg.highByteX << 8) + mainData.allReg.lowByteX;
				mainData.currentY = (mainData.allReg.highByteY << 8) + mainData.allReg.lowByteY;
			}
			break;
		case TLE_TEMP_VALUE:
			{
				bufTX = TLE_UNLOCK_VALUE;
				temp = bufTX >> 8;
				bufTX = (bufTX << 8) + temp;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_TEMP_ENABLE;
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
			 	ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)&mainData.allReg, 7);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_II;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
				ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)&mainData.allReg+7, 7);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_ALL_DATA_III;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 1);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_SPI_Receive(&hspi2, (uint8_t *)&bufRX, 1, 1);
				ret = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)&mainData.allReg+14, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				bufTX = TLE_TEMP_DISABLE;
				temp = bufTX >> 8;
				bufTX = (bufTX << 8) + temp;
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
				ret = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)&bufTX, 2);
				while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

				mainData.temp = (mainData.allReg.highByteX << 8) + mainData.allReg.lowByteX;
			}
			break;

	}

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
				bufTX = TLE_UNLOCK_VALUE;
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
