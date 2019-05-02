/*
 * OJ_Main.h
 *
 *  Created on: 16 ???. 2019 ?.
 *      Author: DIIV
 */

#ifndef INC_OJ_MAIN_H_
#define INC_OJ_MAIN_H_

#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "OJ_TLE.h"

#define PI_CONST 3.1415926535

typedef struct _mainDataTypeDef
{
	float curResultAngle_RAW;
	float curResultAngle_DFT;
	int16_t currentX;
	int16_t currentY;
	int16_t maxX;
	int16_t maxY;
	int16_t minX;
	int16_t minY;
	struct{
		int16_t offsetX;
		int16_t offsetY;
		int16_t amplitudeX;
		int16_t amplitudeY;
		float tempGradX;
		float tempGradY;
		float tempDependX;
		float tempDependY;
	}colibData;
	struct{
		uint8_t	ctrl1;
		uint8_t	lowByteX;
		uint8_t	highByteX;
		uint8_t	lowByteY;
		uint8_t	highByteY;
		uint8_t	fcn_stat;
		uint8_t	fsync_inv;
		uint8_t	angt;
		uint8_t	reserved8h;
		uint8_t	reserved9h;
		uint8_t	reservedAh;
		uint8_t	reservedBh;
		uint8_t	tst;
		uint8_t	deviceID;
		uint8_t	lock;
		uint8_t	ctrl2;
	}allReg;

	int16_t temp;
}mainDataTypeDef;

//typedef struct _mainDataTypeDef
//{
//	uint16_t tleMode;
//}mainDataTypeDef;



typedef struct _sysDataTLETypeDef
{
	int16_t maxX;
	int16_t maxY;
	int16_t minX;
	int16_t minY;
	int16_t tempX;
	int16_t curX;
	int16_t curY;
	float 	ortDef;
	struct{
		float x;
		float y;
	}offset;
	struct{
		float x;
		float y;
	}amplitude;
	struct{
		float x;
		float y;
	}tempGrad;
	struct{
		float x;
		float y;
	}tempDepend;
	struct{
		float x;
		float y;
	}offCor;
	struct{
		float x;
		float y;
	}ampNorm;
	struct{
		float y;
	}nonOrtCor;
	float curResultAngle_RAW;
	float curResultAngle_DFT;
	uint8_t i;
}sysDataTLETypeDef;

void GetMaxMinValues(void);
void GetTempValues(void);
void GetColibValues(void);
float GetResultAngle(void);

#endif /* INC_OJ_MAIN_H_ */
