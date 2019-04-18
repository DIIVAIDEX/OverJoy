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
	double curResultAngle_RAW;
	double curResultAngle_DFT;
	uint8_t i;
}sysDataTLETypeDef;

void GetMaxMinValues(void);
void GetTempValues(void);
void GetColibValues(void);
float GetResultAngle(void);

#endif /* INC_OJ_MAIN_H_ */
