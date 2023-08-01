/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*-----------------------------Begin:General Macro----------------------------*/
#define _SecondsPerMin 60
/*-----------------------------End:General Macro------------------------------*/

/*-----------------------------Begin:Initial Macro----------------------------*/
#define AtHome 0
#define NotAtHome 1
#define IntialFindingSpeed 20
#define AccurateFindingSpeed 2
#define FindingDegreeAboveLimit 180
#define FindingDegreeBelowLimit -180
#define AccurateFindingDegreeAboveLimit 10
#define AccurateFindingDegreeBelowLimit -10
#define IntialState 0
#define IntialStopAndResetState 1
#define AccurateFindingState 2
#define	AccurateStopAndResetState 3
#define EndState 4
/*-----------------------------End:Initial Macro------------------------------*/

/*---------------------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------Begin:PID BLDC Macro---------------------------*/
#define BLDCProportion 														0.2
#define BLDCIntegral 														10
#define BLDCDeltaT 															0.001
#define BLDCClockWise 														1
#define BLDCCounterClockWise 												0
#define BLDCIntegralAboveLimit 												1000
#define BLDCIntegralBelowLimit 											   -1000
#define BLDCSumAboveLimit 													1000
#define BLDCSumBelowLimit 												   -1000
#define _BLDCEncoderPerRound 												800
#define _BLDCGearRatio 														2.5
/*-----------------------------End:PID BLDC Macro-----------------------------*/

/*-----------------------------Begin:PID DC Macro(SPEED)----------------------*/
#define DCProportion 20
#define DCIntegral 500
#define POS 0.001
#define DCDeltaT 0.001
#define DCClockWise -1
#define DCStop 0
#define DCCounterClockWise 1
#define DCIntegralAboveLimit 1000
#define DCIntegralBelowLimit -1000
#define DCSumAboveLimit 1000
#define DCSumBelowLimit -1000
#define _DCEncoderPerRound 68000
/*-----------------------------End:PID DC Macro(SPEED)------------------------*/

/*-----------------------------Begin:PID DC Macro(POS)------------------------*/
#define DCProportionPOS 5
#define DCIntegralPOS 0
#define DCDeltaTPOS 0.001
#define DCSumAboveLimitPOS 1000
#define DCSumBelowLimitPOS -1000
#define FilterAlpha 0
/*-----------------------------End:PID DC Macro(POS)--------------------------*/

/*---------------------------------------------------------------------------------------------------------------------------------------------*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId CalPIDDCHandle;
osThreadId CalPIDBLDCHandle;
osThreadId LogicControlHandle;
/* USER CODE BEGIN PV */
/*---------------------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------Begin:Home Variables----------------------------*/

uint8_t HomeStatus;
uint8_t RunStatus;
uint8_t IntialSpeed;
uint8_t HomeFound;
uint8_t Testcommand;
int TestComand2;

/*-----------------------------End:Home Variables------------------------------*/

/*-----------------------------Begin:Encoder Read Variables--------------------*/

int16_t TimerCounterBLDC;
int XungBLDCX4;
uint16_t Tim1Counter;
int16_t TimerCounterDC;
int XungDCX4,XungDC,PreXung;
double DCDegree,XungTinhToanDC;
int8_t RotateStatus = 1;

/*-----------------------------End:Encoder Read Variables----------------------*/

/*---------------------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------Begin:PID BLDC Variables------------------------*/

double e1,pre_e1;
int Target_value1;
double deltaT1 = BLDCDeltaT; // Sampling Time

double kp1= BLDCProportion,ki1 = BLDCIntegral;

// Khai biến khâu tỉ lệ
double up1;

// khai biến khâu tích phân
double ui1,ui_p1;
int ui_above_limit1 = BLDCIntegralAboveLimit,ui_under_limit1 = BLDCIntegralBelowLimit;

// khai biến output
double u1;
int u_above_limit1 = BLDCSumAboveLimit,u_under_limit1 = BLDCSumBelowLimit;

double _RealVelocity1;
double _FilteredVelocity1;
double _PreviousVelocity1;

uint8_t dir1;
uint16_t pwm1;
/*-----------------------------End:PID BLDC Variables-------------------------*/

/*-----------------------------Begin:PID DC Variables(SPEED)------------------*/

double e2,pre_e2;
double Target_value2;
double deltaT2 = DCDeltaT; // Sampling Time

double kp2= DCProportion,ki2 = DCIntegral;

// Khai biến khâu tỉ lệ
double up2;

// khai biến khâu tích phân
double ui2,ui_p2;
int ui_above_limit2 = DCIntegralAboveLimit,ui_under_limit2 = DCIntegralBelowLimit;

// khai biến output
double u2;
int u_above_limit2 = DCSumAboveLimit,u_under_limit2 = DCSumBelowLimit;
int dir2,pwm2;

double _RealVelocity2;
double _FilteredVelocity2;
double _PreviousVelocity2;
/*-----------------------------End:PID DC Variables(SPEED)---------------------*/

/*-----------------------------Begin:PID DC Variables(POS)---------------------*/
double e3,pre_e3;
double Target_value3;
double deltaT3 = DCDeltaTPOS; // Th�?i gian lấy mẫu

double kp3 = DCProportionPOS , kd3 = DCIntegralPOS;

// Khai biến khâu tỉ lệ
double up3;

// khai biến khâu đạo hàm
double ud3,udf3,udf_p3;
double alpha3 = FilterAlpha; // Hệ số bộ l�?c

// khai biến output
double u3;
int u_above_limit3 = DCSumAboveLimitPOS,u_under_limit3 = DCSumBelowLimit;
/*-----------------------------End:PID DC Variables(POS)-------------------------*/

/*---------------------------------------------------------------------------------------------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartCalPIDDC(void const * argument);
void StartCalPIDBLDC(void const * argument);
void StartLogicControl(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------BEGIN:KalmanFilter-------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
double kalman_filterX(double Xvalue)
{
    float x_k1_k1x,x_k_k1x;
    float Z_kx;
    static float P_k1_k1x;

    static float Qx = 0.0001;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
    static float Rx = 0.005; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
    static float Kgx = 0;
    static float P_k_k1x = 1;

    double kalman_adcx;
    static float kalman_adc_oldx=0;
    Z_kx = Xvalue;
    x_k1_k1x = kalman_adc_oldx;

    x_k_k1x = x_k1_k1x;
    P_k_k1x = P_k1_k1x + Qx;

    Kgx = P_k_k1x/(P_k_k1x + Rx);

    kalman_adcx = x_k_k1x + Kgx * (Z_kx - kalman_adc_oldx);
    P_k1_k1x = (1 - Kgx)*P_k_k1x;
    P_k_k1x = P_k1_k1x;
    kalman_adc_oldx = kalman_adcx;

    return kalman_adcx;
}
//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------END:KalmanFilter---------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//

/*---------------------------------------------------------------------------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------BEGIN: PID MOTOR---------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//
void PIDBLDC(){
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

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------END: PID MOTOR-----------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//



/*---------------------------------------------------------------------------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------BEGIN: DRIVE MOTOR-------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//

void DriveBLDC(uint8_t Dir,uint16_t PWM)
{
	HAL_GPIO_WritePin(DirBLDC_GPIO_Port, DirBLDC_Pin, Dir);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,PWM);
}

void DriveDC(int Dir,uint16_t PWM)
{
	if(Dir>0)
	{
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,1000-PWM);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,1000);
	}
	else if(Dir < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,1000);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,1000-PWM);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,1000);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,1000);
	}
}

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------END: DRIVE MOTOR---------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------BEGIN: RESET MOTOR-------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//

void ResetDegree(uint8_t ResetEnable){
	if(ResetEnable == 1){
		TIM3 -> CNT = 0;
		XungDCX4 = 0;
	}
}

void SetAndResetU2parameter(uint8_t command){
	if(!command){
		ui_above_limit2 = 0;
		ui_under_limit2 = 0;
		u_above_limit2 = 0;
		u_under_limit2 =0;
		_PreviousVelocity2 = 0;
		_RealVelocity2 = 0;
	}else{
		ui_above_limit2 = DCIntegralAboveLimit;
		ui_under_limit2 = DCIntegralBelowLimit;
		u_above_limit2 = DCSumAboveLimit;
		u_under_limit2 = DCSumBelowLimit;
	}
}

void SetAndResetU1parameter(uint8_t command){
	if(!command){
		ui_above_limit1 = 0;
		ui_under_limit1 = 0;
		u_above_limit1 = 0;
		u_under_limit1 =0;
		_PreviousVelocity1 = 0;
		_RealVelocity1 = 0;
		Target_value1 = 0;
	}else{
		ui_above_limit1 = BLDCIntegralAboveLimit;
		ui_under_limit1 = BLDCIntegralBelowLimit;
		u_above_limit1 = BLDCSumAboveLimit;
		u_under_limit1 = BLDCSumBelowLimit;
	}
}

void StartBldc(){
	SetAndResetU1parameter(0);
	osDelay(10);
	SetAndResetU1parameter(1);
}

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------END: RESET MOTOR---------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------BEGIN: Home Finding------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//


void HomeFinding(){
  // Reading home sensor if the  wheel at home or not
  // if at home the wheel won't run and set the initial Degree to 0
  // if not the wheel will find home
  if(HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin)== AtHome){
	osDelay(1);
	if(HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin)== AtHome){
		HomeStatus = 1;
		if((RunStatus == IntialState)||(RunStatus == AccurateFindingState)){
			Target_value3 = DCDegree;
			RunStatus++;
		}
	  }
	}

  // At this State the Wheel will find its home at high speed
  // And will reserve when it reach the Degree limits not to break the wires
  if(RunStatus == IntialState){
	if(DCDegree > FindingDegreeAboveLimit){
		RotateStatus = DCClockWise;
	}else if(DCDegree< FindingDegreeBelowLimit){
		RotateStatus = DCCounterClockWise;
	}
	Target_value2 = IntialFindingSpeed*RotateStatus;
  }

 //At this State the Wheel will stop and Reset to 0 Degree
if((RunStatus == IntialStopAndResetState)||(RunStatus == AccurateStopAndResetState)){
	osDelay(500);
	SetAndResetU2parameter(0);
	ResetDegree(1);
	Target_value3 = 0;
	osDelay(10);
	ResetDegree(0);
	SetAndResetU2parameter(1);
	RunStatus += 1;
}

//At this State the wheel will run at low speed to find its home
//And also reserve when its reach the degree limits
if ((RunStatus == AccurateFindingState)&&(HAL_GPIO_ReadPin(Home_GPIO_Port, Home_Pin) == NotAtHome)){
	if(DCDegree < AccurateFindingDegreeBelowLimit){
		RotateStatus = DCCounterClockWise;
	}else if(DCDegree > AccurateFindingDegreeAboveLimit){
		RotateStatus = DCClockWise;
	}
	Target_value2 = AccurateFindingSpeed*RotateStatus;
	}
//End Home Finding
if (RunStatus == EndState){
	HomeFound = 1;
}
}

//-----------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------END: Home Finding--------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------//


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
//  HAL_TIM_Base_Start_IT(&htim1);

  //Set intial Speed of both Motor as 0 RPM
  DriveBLDC(0, 0);
  DriveDC(0,0);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CalPIDDC */
  osThreadDef(CalPIDDC, StartCalPIDDC, osPriorityAboveNormal, 0, 128);
  CalPIDDCHandle = osThreadCreate(osThread(CalPIDDC), NULL);

  /* definition and creation of CalPIDBLDC */
  osThreadDef(CalPIDBLDC, StartCalPIDBLDC, osPriorityBelowNormal, 0, 128);
  CalPIDBLDCHandle = osThreadCreate(osThread(CalPIDBLDC), NULL);

  /* definition and creation of LogicControl */
  osThreadDef(LogicControl, StartLogicControl, osPriorityNormal, 0, 128);
  LogicControlHandle = osThreadCreate(osThread(LogicControl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DirBLDC_GPIO_Port, DirBLDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DirBLDC_Pin */
  GPIO_InitStruct.Pin = DirBLDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DirBLDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Home_Pin */
  GPIO_InitStruct.Pin = Home_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Home_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCalPIDDC */
/**
  * @brief  Function implementing the CalPIDDC thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_StartCalPIDDC */
void StartCalPIDDC(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {


//	if((HomeStatus)&&(RunStatus != AccurateFindingState)){
		PIDDCPos();
//	}
//	else{
//		PIDDCSpeed();
//	}
	DriveDC(dir2, pwm2);
	osDelay(1);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCalPIDBLDC */
/**
* @brief Function implementing the CalPIDBLDC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCalPIDBLDC */
void StartCalPIDBLDC(void const * argument)
{
  /* USER CODE BEGIN StartCalPIDBLDC */
  /* Infinite loop */
  for(;;)
  {
	PIDBLDC();
	DriveBLDC(dir1, pwm1);
	osDelay(1);
  }
  /* USER CODE END StartCalPIDBLDC */
}

/* USER CODE BEGIN Header_StartLogicControl */
/**
* @brief Function implementing the LogicControl thread.
* @param argument: Not used
* @retval None
*/

int TestDegree[] = {90,180,50,0,60,0,-90,10};
int TestSpeed[] = {100,0,120,200,100,200,100,0};
/* USER CODE END Header_StartLogicControl */
void StartLogicControl(void const * argument)
{
  /* USER CODE BEGIN StartLogicControl */
  /* Infinite loop */
  for(;;)
  {
//	if(HomeFound == 0)
//	{
//		HomeFinding();
//	}
//	if(Testcommand == 1){
//		StartBldc();
//		Testcommand = 0;
//	}
//	if(TestComand2 == 1)
//	  {
//		  for(int i = 0;i<8;i++){
//			  Target_value1 = TestSpeed[i];
//			  Target_value3 = TestDegree[i];
//			  osDelay(1000);
//		  }
//	  }
    osDelay(1);
  }


  /* USER CODE END StartLogicControl */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
