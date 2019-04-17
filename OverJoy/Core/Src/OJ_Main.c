/*
 * OJ_Main.c
 *
 *  Created on: 16 ???. 2019 ?.
 *      Author: DIIV
 */


#include "OJ_Main.h"

sysDataTLETypeDef sysDataTLE;
uint16_t tleValues[2];


void GetMaxMinValues(void)
{
	TLEReadData(TLE_MAIN_VALUES, tleValues);
	if(sysDataTLE.maxX == 0 && sysDataTLE.maxY == 0){
		sysDataTLE.maxX = tleValues[0];
		sysDataTLE.maxY = tleValues[1];
	}
	else{
		sysDataTLE.minX = tleValues[0];
		sysDataTLE.minY = tleValues[1];
		GetColibValues();
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
	TLEReadData(TLE_TEMP_VALUE, &tleValues);
	sysDataTLE.tempDepend.x = sysDataTLE.offset.x + (sysDataTLE.tempGrad.x / -188.75) * (tleValues[0] - sysDataTLE.tempX);
	sysDataTLE.tempDepend.y = sysDataTLE.offset.y + (sysDataTLE.tempGrad.y / -188.75) * (tleValues[0] - sysDataTLE.tempX);

}

float ResultAngle(void)
{
	float resultAngle;

	TLEReadData(TLE_MAIN_VALUES, tleValues);

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
	sysDataTLE.nonOrtCor.y = (sysDataTLE.ampNorm.y - sysDataTLE.ampNorm.x * sin(tleValues[1])) / cos(tleValues[0]);

	/*		Resulting Angle
	================================================================*/
	return resultAngle = atan(sysDataTLE.nonOrtCor.y / sysDataTLE.ampNorm.x) - cos(tleValues[0]);
}
