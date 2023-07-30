#include "PID.h"
double PID_cal(PID_Param *pid)
{
    pidEnCount = __HAL_TIM_GET_COUNTER(pid_htim);
    __HAL_TIM_SET_COUNTER(pid_htim,0);
	if(!pid) return -1;
	double alpha=0;
	e = setVal - currVal;
	uP = kp*e;
	uI = uI_P + ki*e*deltaT;
	uI = uI > uI_HLim ? uI_HLim : 
	    (uI < uI_LLim ? uI_LLim : uI);
	uD = kd*(e - e_p)/deltaT;
	uDF = (1-alpha)*uDF_P+alpha*uD;
	e_p = e;
	uI_P = uI;
	uDF_P = uDF;
	uCurr = uP+uI+uDF;
	uCurr =  uCurr > u_HLim ? u_HLim : 
	   		(uCurr < u_LLim ? u_LLim : uCurr);
	return uCurr;
}

void PIDBLDC()
{
	//Calculate the speed of BLDC
	TimerCounterBLDC = __HAL_TIM_GET_COUNTER(&htim4);
	XungBLDCX4 += TimerCounterBLDC;
	TIM4 -> CNT = 0;
	_RealVelocity1 = ((XungBLDCX4/deltaT1)/(_BLDCEncoderPerRound*_BLDCGearRatio))*_SecondsPerMin;
	_FilteredVelocity1 = 0.854 * _FilteredVelocity1 + 0.0728 * _RealVelocity1+ 0.0728 * _PreviousVelocity1;
	_PreviousVelocity1 = _RealVelocity1;
	XungBLDCX4=0;

	// khoảng cần đáp ứng của hệ thống
	e1 = Target_value1 - _RealVelocity1;

	// Khâu tỉ lệ
	up1 = kp1*e1;

	// khâu tích phân
	ui1 = ui_p1 + ki1*e1*deltaT1;
	// Bão hòa khâu tích phân
	if (ui1>ui_above_limit1)ui1=ui_above_limit1;
	else if(ui1<ui_under_limit1)ui1=ui_under_limit1;

	pre_e1 = e1;
	ui_p1 = ui1;

	// Tổng out put
	u1 = up1 + ui1;
	// Bão hòa output
	if(u1>u_above_limit1)u1 = u_above_limit1;
	else if (u1<u_under_limit1)u1=u_under_limit1;

	// Xác định chi�?u:
	if (u1>0)dir1=BLDCClockWise;
	else if(u1<=0)dir1 = BLDCCounterClockWise;

	pwm1 = abs(u1);
}

void PIDDCSpeed()
{
	//Calculate velocity of DC Servo:
	TimerCounterDC = __HAL_TIM_GET_COUNTER(&htim3);
	XungDCX4 += TimerCounterDC;
	__HAL_TIM_SET_COUNTER(&htim3,0);
	XungDC = XungDCX4/(4);
	XungTinhToanDC = XungDC;
	DCDegree = XungTinhToanDC*360/_DCEncoderPerRound;

	_RealVelocity2 = (((XungDC-PreXung)/deltaT2)/(_DCEncoderPerRound))*_SecondsPerMin;
//	_FilteredVelocity2 = 0.854 * _FilteredVelocity2 + 0.0728 * _RealVelocity2+ 0.0728 * _PreviousVelocity2;
//	_PreviousVelocity2 = _RealVelocity2;
	PreXung = XungDC;
	// khoảng cần đáp ứng của hệ thống
	e2 = Target_value2 - _RealVelocity2;

	// Khâu tỉ lệ
	up2 = kp2*e2;

	// khâu tích phân
	ui2 = ui_p2 + ki2*e2*deltaT2;
	// Bão hòa khâu tích phân
	if (ui2>ui_above_limit2)ui2=ui_above_limit2;
	else if(ui2<ui_under_limit2)ui2=ui_under_limit2;

	pre_e2 = e2;
	ui_p2 = ui2;

	// Tổng out put
	u2 = up2 + ui2;
	// Bão hòa output
	if(u2>u_above_limit2)u2 = u_above_limit2;
	else if (u2<u_under_limit2)u2=u_under_limit2;

	// Xác định chi�?u:
	if (u2>0)dir2=DCClockWise;
	else if(u2<0)dir2 = DCCounterClockWise;
	else dir2 = DCStop;

	pwm2 = abs(u2);

}

void PIDDCPos(){
	// khoảng cần đáp ứng của hệ thống
	e3 = Target_value3 - DCDegree;

	// Khâu tỉ lệ
	up3 = kp3*e3;

	// khâu đạo hàm
//	ud3 = kd3*(e3 - pre_e3)/deltaT3;
//	// L�?c thông thấp khâu đạo hàm
//	udf3 = (1-alpha3)*udf_p3+alpha3*ud3;

//	pre_e3 = e3;
//	udf_p3 = udf3;

	// Tổng out put
	u3 = up3 ;//+ udf3;
	// Bão hòa output
	if(u3>u_above_limit3)u3 = u_above_limit3;
	else if (u3<u_under_limit3)u3=u_under_limit3;
	// Xuất giá trị Target
	Target_value2 = u3;

	//Lồng PID Vận Tốc
	PIDDCSpeed();

}
