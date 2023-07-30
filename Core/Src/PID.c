/*
 * PID.c
 *
 *  Created on: Jul 30, 2023
 *      Author: LENOVO
 */

#include "PID.h"

//-----------------------------------------------Begin: Setting Parameter for PID------------------------------------------//
void Pid_SetParam(PID_Param *pid,double kP,double kI,double kD,double deltaT,double uI_AboveLimit,double uI_BelowLimit,double u_AboveLimit,double u_BelowLimit)
{
//----------------------Term-----------------------//
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
//----------------------Sample Time----------------//
	pid->deltaT = deltaT;
//----------------------Limit----------------------//
	pid->uI_AboveLimit = uI_AboveLimit;
	pid->uI_BelowLimit = uI_BelowLimit;
	pid->u_AboveLimit = u_AboveLimit;
	pid->u_BelowLimit = u_BelowLimit;
}

//-----------------------------------------------End: Setting Parameter for PID--------------------------------------------//

//------------------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------Begin: Change Parameter for PID-------------------------------------------//

void Pid_uI_PreSetParam(PID_Param *pid,double uI_AboveLimit,double uI_BelowLimit)
{
	pid->uI_AboveLimit = uI_AboveLimit;
	pid->uI_BelowLimit = uI_BelowLimit;
}



void Pid_u_PresetParam(PID_Param *pid,double u_AboveLimit,double u_BelowLimit)
{
	pid->u_AboveLimit = u_AboveLimit;
	pid->u_BelowLimit = u_BelowLimit;
}



void Pid_Term_PresetParam(PID_Param *pid,double kP,double kI,double kD)
{
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
}

//-----------------------------------------------End: Change Parameter for PID---------------------------------------------//

//------------------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------Begin: Calculating PID---------------------------------------------------//

double Pid_Cal(PID_Param *pid,double Target)
{
//-----------------------Input-------------------------//
	pid->Target = Target;
	pid->e = pid->Target - pid->CurrVal;

//-----------------------Propotion Term----------------//
	pid->uP = pid->kP*pid->e;

//-----------------------Integral Term-----------------//
	pid->uI = pid->uI_Pre + pid->kI*pid->e*pid->deltaT;
	pid->uI = pid->uI > pid->uI_AboveLimit ? pid->uI_AboveLimit : pid->uI;
	pid->uI = pid->uI < pid->uI_BelowLimit ? pid->uI_BelowLimit : pid->uI;

//-----------------------Derivative Term---------------//
	pid->uD = pid->kD*(pid->e - pid->e_Pre)/pid->deltaT;
	pid->uD_Fil = (1-pid->alpha)*pid->uD_FilPre+pid->alpha*pid->uD;

//-----------------------Previous Value----------------//
	pid->e_Pre = pid->e;
	pid->uI_Pre = pid->uI;
	pid->uD_FilPre = pid->uD_Fil;

//-----------------------Sum---------------------------//
	pid->u = pid->uP + pid->uI + pid->uD;
	pid->u = pid->u > pid->u_AboveLimit ? pid->u_AboveLimit : pid->u;
	pid->u = pid->u < pid->u_BelowLimit ? pid->u_BelowLimit : pid->u;

	return pid->u;
}

//-----------------------------------------------End: Calculating PID-----------------------------------------------------//

void EncoderSetting(EncoderRead *enc,TIM_HandleTypeDef *htim,int count_PerRevol,double deltaT)
{
	enc->htim = htim;
	enc->count_PerRevol = count_PerRevol;
	enc->deltaT = deltaT;
}
void SpeedReadOnly(EncoderRead *enc)
{
	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->vel_Real = (enc->count_X4/enc->deltaT)/(enc->count_PerRevol)*60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real+ 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_X4 = 0;
}



void SpeedReadNonReset(EncoderRead *enc){

	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);
	enc->vel_Real = ((enc->count_X4-enc->count_Pre)/enc->deltaT)/(enc->count_PerRevol)*60;
	enc->vel_Fil = 0.854 * enc->vel_Fil + 0.0728 * enc->vel_Real+ 0.0728 * enc->vel_Pre;
	enc->vel_Pre = enc->vel_Real;
	enc->count_Pre = enc->count_X4;
}



double CountRead(EncoderRead *enc,uint8_t count_mode){
	enc->count_PerRevol = enc->count_PerRevol;
	enc->count_Timer = __HAL_TIM_GET_COUNTER(enc->htim);
	enc->count_X4 += enc->count_Timer;
	__HAL_TIM_SET_COUNTER(enc->htim,0);

	if (enc->count_Mode == ModeX4)
	{
		return enc->count_X4;
	}else if (enc->count_Mode == ModeX1)
	{
		enc->count_X1 = enc->count_X4/4;
		return enc->count_X1;
	}else if (enc->count_Mode == ModeDegree)
	{
		enc->Degree = enc->count_X4*360/enc->count_PerRevol;
		return enc->Degree;
	}else {
		return 0;
	}
}


