/*
 * OJ_Main.c
 *
 *  Created on: 16 ???. 2019 ?.
 *      Author: DIIV
 */


#include "OJ_Main.h"

sysDataTLETypeDef sysDataTLE;
int16_t tleValues[2];


void GetMaxMinValues(void)
{
	TLEReadData(TLE_MAIN_VALUES, tleValues);

	if(sysDataTLE.maxX < tleValues[0]){
		sysDataTLE.maxX = tleValues[0];
	}
	if(sysDataTLE.minX > tleValues[0]){
		sysDataTLE.minX = tleValues[0];
	}

	if(sysDataTLE.maxY < tleValues[1]){
		sysDataTLE.maxY = tleValues[1];
	}
	if(sysDataTLE.minY > tleValues[1]){
		sysDataTLE.minY = tleValues[1];
	}
}

void GetTempValues(void)
{
	TLEReadData(TLE_TEMP_VALUE, &tleValues);
	sysDataTLE.tempX = tleValues[0];
//	sysDataTLE.tempY = tleValues[1];
}


void GetColibValues(void)
{
	/*		Offset Definition
	================================================================*/
	sysDataTLE.offset.x = (sysDataTLE.maxX + sysDataTLE.minX) / 2;
	sysDataTLE.offset.y = (sysDataTLE.maxY + sysDataTLE.minY) / 2;

	/*		Amplitude Definition
	================================================================*/
	sysDataTLE.amplitude.x = (sysDataTLE.maxX - sysDataTLE.minX) / 2;
	sysDataTLE.amplitude.y = (sysDataTLE.maxY - sysDataTLE.minY) / 2;

	/*		Temperature-Dependent Behavior
	================================================================*/
	sysDataTLE.tempGrad.x = 0.116296 + (0.0010147 * sysDataTLE.offset.x);
	sysDataTLE.tempGrad.y = -0.079401 + (0.0010121 * sysDataTLE.offset.y);
	/*		Temperature-Dependent Offset Value
	================================================================*/
//	TLEReadData(TLE_TEMP_VALUE, &tleValues);
//	sysDataTLE.tempDepend.x = sysDataTLE.offset.x + (sysDataTLE.tempGrad.x / -188.75) * (tleValues[0] - sysDataTLE.tempX);
//	sysDataTLE.tempDepend.y = sysDataTLE.offset.y + (sysDataTLE.tempGrad.y / -188.75) * (tleValues[0] - sysDataTLE.tempX);
	sysDataTLE.tempDepend.x = sysDataTLE.offset.x + (sysDataTLE.tempGrad.x / -188.75);
	sysDataTLE.tempDepend.y = sysDataTLE.offset.y + (sysDataTLE.tempGrad.y / -188.75);

	/*		Temperature-Dependent Offset Value
	================================================================*/
//	TLEReadData(TLE_MAIN_VALUES, tleValues);
	sysDataTLE.ortDef = cos(tleValues[0]) - sin(tleValues[1]);
}

float GetResultAngle(void)
{
	float radAngle;

	TLEReadData(TLE_MAIN_VALUES, tleValues);
	sysDataTLE.curX = tleValues[0];
	sysDataTLE.curY = tleValues[1];

	/*		Offset Correction
	================================================================*/
	sysDataTLE.offCor.x = tleValues[0] - sysDataTLE.tempDepend.x;
	sysDataTLE.offCor.y = tleValues[1] - sysDataTLE.tempDepend.y;

	/*		Amplitude Normalization
	================================================================*/
	sysDataTLE.ampNorm.x = sysDataTLE.offCor.x / sysDataTLE.amplitude.x;
	sysDataTLE.ampNorm.y = sysDataTLE.offCor.y / sysDataTLE.amplitude.y;

	/*		Non-Orthogonality Correction
	================================================================*/
	sysDataTLE.nonOrtCor.y = (sysDataTLE.ampNorm.y - sysDataTLE.ampNorm.x * sin(1.2400616409267344)) / cos(1.2400616409267344);

	/*		Resulting Angle
	================================================================*/
	radAngle = atan2(sysDataTLE.ampNorm.y, sysDataTLE.ampNorm.x);
	if(radAngle >= -PI_CONST && radAngle <= 0){
		radAngle = PI_CONST + (PI_CONST + radAngle);
	}
	sysDataTLE.curResultAngle_RAW = (radAngle * 180) / PI_CONST;

	radAngle = atan2(sysDataTLE.nonOrtCor.y, sysDataTLE.ampNorm.x) - -1.8997343902912205;
	if(radAngle >= -PI_CONST && radAngle <= 0){
		radAngle = PI_CONST + (PI_CONST + radAngle);
	}
	sysDataTLE.curResultAngle_DFT = (radAngle * 180) / PI_CONST;

	return 0;
}

void OrtCorrByDFT(void)
{
	sysDataTLE.i = 0;
	int16_t arrayValuesX[64];
	int16_t arrayValuesY[64];
	double dftXr = 0;
	double dftXi = 0;
	double dftYr = 0;
	double dftYi = 0;
	volatile double aX = 0;
	volatile double aY = 0;
	volatile double fi = 0;
	volatile double fiX = 0;
	volatile double fiY = 0;

	float startAnglePos;

	GetResultAngle();
	startAnglePos = 0;

	while(sysDataTLE.i < 64){
		GetResultAngle();
//		if(startAnglePos + sysDataTLE.i* 5.625 >= 360){
//			startAnglePos = 360 - startAnglePos;
//		}
		if(startAnglePos + sysDataTLE.i* 5.625 > sysDataTLE.curResultAngle_RAW - 0.05 && startAnglePos + sysDataTLE.i* 5.625 < sysDataTLE.curResultAngle_RAW + 0.05){
			arrayValuesX[sysDataTLE.i] = sysDataTLE.curX;
			arrayValuesY[sysDataTLE.i] = sysDataTLE.curY;
			sysDataTLE.i++;
		}
	}

	for(uint8_t i = 0; i < 64; i++){
		dftXr += arrayValuesX[i] * cos(i * 5.625);
		dftXi += arrayValuesX[i] * sin(i * 5.625);
		dftYr += arrayValuesY[i] * cos(i * 5.625);
		dftYi += arrayValuesY[i] * sin(i * 5.625);
	}

	dftXr = (dftXr * 2) / 64;
	dftXi = (dftXi * 2) / 64;
	dftYr = (dftYr * 2) / 64;
	dftYi = (dftYi * 2) / 64;
	///////////////////////////////////////////
	fiX = atan2(dftXi, dftXr);
	fiY = PI_CONST/2 - atan2(dftYi, dftYr);
	fi = fiX - fiY;
	///////////////////////////////////////////
	dftXr = pow(dftXr, 2);
	dftXi = pow(dftXi, 2);
	dftYr = pow(dftYr, 2);
	dftYi = pow(dftYi, 2);

	aX = sqrt(dftXr + dftXi);
	aY = sqrt(dftYr + dftYi);

	fi = PI_CONST/2 - atan2(dftYi, dftYr);

	__NOP();
}
