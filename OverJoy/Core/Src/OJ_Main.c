/*
 * OJ_Main.c
 *
 *  Created on: 16 ???. 2019 ?.
 *      Author: DIIV
 */


#include "OJ_Main.h"

sysDataTLETypeDef sysDataTLE;
mainDataTypeDef mainData;

int16_t tleValues[2];

uint8_t tempArray[64];

void GetMaxMinValues(void)
{
	if(mainData.maxX < mainData.currentX){
		mainData.maxX = mainData.currentX;
	}
	if(mainData.minX > mainData.currentX){
		mainData.minX = mainData.currentX;
	}

	if(mainData.maxY < mainData.currentY){
		mainData.maxY = mainData.currentY;
	}
	if(mainData.minY > mainData.currentY){
		mainData.minY = mainData.currentY;
	}
}

void GetTempValues(void)
{
	TLEReadData(TLE_TEMP_VALUE, NULL);
}


void GetColibValues(void)
{
	/*		Offset Definition
	================================================================*/
	mainData.colibData.offsetX = (mainData.maxX + mainData.minX) / 2;
	mainData.colibData.offsetY = (mainData.maxY + mainData.minY) / 2;

	/*		Amplitude Definition
	================================================================*/
	mainData.colibData.amplitudeX = (mainData.maxX - mainData.minX) / 2;
	mainData.colibData.amplitudeY = (mainData.maxY - mainData.minY) / 2;

	/*		Temperature-Dependent Behavior
	================================================================*/
	mainData.colibData.tempGradX = 0.116296 + (0.0010147 * mainData.colibData.offsetX);
	mainData.colibData.tempGradY = -0.079401 + (0.0010121 * mainData.colibData.offsetY);
	/*		Temperature-Dependent Offset Value
	================================================================*/
//	TLEReadData(TLE_TEMP_VALUE, &tleValues);
//	sysDataTLE.tempDepend.x = sysDataTLE.offset.x + (sysDataTLE.tempGrad.x / -188.75) * (tleValues[0] - sysDataTLE.tempX);
//	sysDataTLE.tempDepend.y = sysDataTLE.offset.y + (sysDataTLE.tempGrad.y / -188.75) * (tleValues[0] - sysDataTLE.tempX);
	mainData.colibData.tempDependX = mainData.colibData.offsetX + (mainData.colibData.tempGradX / -188.75);
	mainData.colibData.tempDependY = mainData.colibData.offsetY + (mainData.colibData.tempGradY / -188.75);

//	/*		Temperature-Dependent Offset Value
//	================================================================*/
////	TLEReadData(TLE_MAIN_VALUES, tleValues);
//	sysDataTLE.ortDef = cos(tleValues[0]) - sin(tleValues[1]);
}

float GetResultAngle(void)
{
	float offCorX;
	float offCorY;
	float ampNormX;
	float ampNormY;
	float radAngle;
	static float fi = 0;
	static float fiX = 0;

	TLEReadData(TLE_MAIN_VALUES, NULL);
	GetMaxMinValues();
	GetColibValues();


	/*		Offset Correction
	================================================================*/
	offCorX = mainData.currentX - mainData.colibData.tempDependX;
	offCorY = mainData.currentY - mainData.colibData.tempDependY;

	/*		Amplitude Normalization
	================================================================*/
	ampNormX = offCorX / mainData.colibData.amplitudeX;
	ampNormY = offCorY / mainData.colibData.amplitudeY;

	/*		Resulting Angle RAW
	================================================================*/
	radAngle = atan2(ampNormY, ampNormX);
	if(radAngle >= -PI_CONST && radAngle <= 0){
		radAngle = PI_CONST + (PI_CONST + radAngle);
	}
	mainData.curResultAngle_RAW = (radAngle * 180) / PI_CONST;

	/*		Resulting Angle With Non-Orthogonality Correction
	================================================================*/
//	if(fi != 0 && fiX != 0){
//		sysDataTLE.nonOrtCor.y = (sysDataTLE.ampNorm.y - sysDataTLE.ampNorm.x * sin(fi)) / cos(fi);
//		radAngle = atan2(sysDataTLE.nonOrtCor.y, sysDataTLE.ampNorm.x) - mainData.colibData.fiX;
//		if(radAngle >= -PI_CONST && radAngle <= 0){
//			radAngle = PI_CONST + (PI_CONST + radAngle);
//		}
//		mainData.curResultAngle_DFT = (radAngle * 180) / PI_CONST;
//	}
	return 0;
}

/*		DFT method
 *  Try to correct phase offset (orthogonality error) basing on
 *  itself's corrupted data :D
================================================================*/
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

	float startAnglePos = 0;

	GetResultAngle();

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
