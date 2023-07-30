/*
 * PID.h
 *
 *  Created on: Jul 30, 2023
 *      Author: LENOVO
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

//------------------------Begin: Struct of OutSumValue-----------------------------//
typedef struct PID_Param{
//---------Input Parameters-----------//
	double Target;
	double CurrVal;
	double e;
	double e_Pre;
	double deltaT;
//---------Propotion Parameters-------//
	double kP;
	double uP;
//---------Intergral Parameters-------//
	double kI;
	double uI;
	double uI_Pre;
	int uI_AboveLimit;
	int uI_BelowLimit;
//---------Derivative Parameters------//
	double kD;
	double uD;
	double uD_Fil;
	double uD_FilPre;
	double alpha;
//---------Sum Parameters-------------//
	double u;
	double u_AboveLimit;
	double u_BelowLimit;
}PID_Param;
//------------------------End: Struct of OutSumValue-------------------------------//

//------------------------Begin: Struct of Encoder Read----------------------------//

typedef struct EncoderRead{
//------------------------Timer & Count--------------//
	TIM_HandleTypeDef *htim;
	int16_t count_Timer ;
	int count_X4;
	int count_X1;
	int count_Pre;
	int count_PerRevol;
	uint8_t count_Mode;
//------------------------Speed Val-----------------//
	double vel_Real;
	double vel_Pre;
	double vel_Fil;
//------------------------Pos Cal-------------------//
	double Degree;
	double deltaT;
}EncoderRead;

//------------------------End: Struct of Encoder Read------------------------------//


//------------------------Begin: Function of Pid-----------------------------------//
void Pid_SetParam(PID_Param *pid,double kP,double kI,double kD,double deltaT,double uI_AboveLimit,double uI_BelowLimit,double u_AboveLimit,double u_UnderLimit);
void Pid_uI_PreSetParam(PID_Param *pid,double uI_AboveLimit,double uI_BelowLimit);
void Pid_u_PresetParam(PID_Param *pid,double u_AboveLimit,double u_BelowLimit);
void Pid_Term_PresetParam(PID_Param *pid,double kP,double kI,double kD);
double Pid_Cal(PID_Param *pid,double Target);
//------------------------End: Function of Pid-------------------------------------//


#define ModeX1 0
#define ModeX4 1
#define ModeDegree 2

void EncoderSetting(EncoderRead *enc,TIM_HandleTypeDef *htim,int count_PerRevol,double deltaT);
void SpeedReadOnly(EncoderRead *enc);
void SpeedReadNonReset(EncoderRead *enc);
double CountRead(EncoderRead *enc,uint8_t count_mode);
#endif /* INC_PID_H_ */
