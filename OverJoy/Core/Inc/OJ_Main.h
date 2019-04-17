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

typedef struct _sysDataTLETypeDef
{
	uint16_t maxX;
	uint16_t maxY;
	uint16_t minX;
	uint16_t minY;
	uint16_t tempX;
	uint16_t tempY;
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
}sysDataTLETypeDef;

void GetMaxMinValues(void);
void GetTempValues(void);
void GetColibValues(void);
float ResultAngle(void);

#endif /* INC_OJ_MAIN_H_ */
