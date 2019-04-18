/*
 * OJ_TLE.h
 *
 *  Created on: 16 ???. 2019 ?.
 *      Author: DIIV
 */

#ifndef INC_OJ_TLE_H_
#define INC_OJ_TLE_H_

#include "OJ_Main.h"

#define TLE_MAIN_VALUES 0x01
#define TLE_TEMP_VALUE 	0x02

#define TLE_CFG_INIT 	0x10

#define TLE_UPDATE			0x0000
#define TLE_ALL_DATA_I		0x0087
#define TLE_ALL_DATA_II		0x00BF
#define TLE_ALL_DATA_III	0x00F2

typedef struct _dataStructTLETypeDef
{
	uint8_t	ctrl1;
	int16_t valueX;
	int16_t valueY;
	uint8_t	fcn_stat;
	uint8_t	fsync_inv;
	uint8_t	angt;
	uint8_t	reserved1;
	uint8_t	reserved2;
	uint8_t	reserved3;
	uint8_t	reserved4;
	uint8_t	tst;
	uint8_t	deviceID;
	uint8_t	lock;
	uint8_t	ctrl2;


}dataStructTLETypeDef;

uint8_t TLEReadData(uint8_t dataType, void *pData);
uint8_t TLEConfig(uint8_t confType, void *pData);

#endif /* INC_OJ_TLE_H_ */
