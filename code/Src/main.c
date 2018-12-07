/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_COMP2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void SwitchStep(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

volatile int startup_time = 0;



#include <stdarg.h>	// for va_start(lst, fmt), va_end(lst);...
#include <string.h> //for strlen()

static char buf[256];
void Printf(const char *fmt, ...)
{
//	static char buf[256];
	char *p;
	va_list lst;

	va_start(lst, fmt);
	vsprintf(buf, fmt, lst);
	va_end(lst);

	p = buf;
	HAL_UART_Transmit_DMA(&huart1, (unsigned char *)buf, strlen(buf));
	return;
	while(*p) {
		HAL_UART_Transmit(&huart1, (unsigned char *)p, 1, 500);
		p++;
	}
}


int ChangeTime = 0;
int TimeRotate = 0;

int incorrectfl = 0;
int correctfl = 0;


#define CounterTIM3Read htim3.Instance->CNT				// For Input pin. for count impulse lenght.
#define CounterTIM2Read htim2.Instance->CNT				// Counter zc_polarity and time for STEPs
#define CounterTIM1Read htim1.Instance->CNT				// Counter us After Impulses
#define CounterTIM6Read htim6.Instance->CNT				// Timer Clocker impulse N-Mosfet (Low-Mosfet)

#define COMMUTATION_TIMING_IIR_COEFF_A      1
#define COMMUTATION_TIMING_IIR_COEFF_B      20//50

#define STURTUP_TIME 3000//5000//2600

#define COMP_CHN_A             1
#define COMP_CHN_B             2
#define COMP_CHN_C             3

unsigned char COMP_CHN_Table[6];

volatile int   TIME_IMPULS = 0;		//10
#define  TIME_PERIOD  120	//60

struct driveTableStructure{
	GPIO_TypeDef* GPIO_Port;
	uint16_t GPIO_Pin;
};

struct driveTableStructure driveTableHight[6], driveTableLow[6];


volatile unsigned int filteredTimeSinceCommutation;
volatile unsigned char nextCommutationStep;
volatile unsigned char zcPolarity;


int timeSinceCommutation_Updated = 0;
int timeSinceCommutation_Value;


volatile int Impulse_Ready = 0;
volatile int Impulse_Length = 0;

#define IMP_AVERAGE 10
int Impulse_average[IMP_AVERAGE] = {0,};
int Impulse_Average_arr=0;
int Impulse_Average=0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(TEST_IN_GPIO_Port, TEST_IN_Pin) == GPIO_PIN_RESET){
		int timer3 = CounterTIM3Read;
		if (timer3 < 1200) Impulse_Length=0;
		else Impulse_Length = (timer3-1000)/8;

		Impulse_average[Impulse_Average_arr]=Impulse_Length;

		if (++Impulse_Average_arr>=IMP_AVERAGE)Impulse_Average_arr=0;

		int i=0;
		for (i=0; i<IMP_AVERAGE; i++){Impulse_Average +=Impulse_average[i];}
		Impulse_Average/=IMP_AVERAGE;

		if (Impulse_Length == 0)TIME_IMPULS=0;
		else TIME_IMPULS = Impulse_Average;

		Impulse_Ready = 1;
	}

	CounterTIM3Read = 0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}


void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
}


void HAL_COMP_MspInit_Add(COMP_HandleTypeDef* hcomp)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hcomp->Instance==COMP2)
  {
  /* USER CODE BEGIN COMP2_MspInit 0 */

  /* USER CODE END COMP2_MspInit 0 */

    /**COMP2 GPIO Configuration
    PA3     ------> COMP2_INP
    PA4     ------> COMP2_INM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* COMP2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
  /* USER CODE BEGIN COMP2_MspInit 1 */

  /* USER CODE END COMP2_MspInit 1 */
  }

}


void MX_COMP2_Init_Channel(int n, int edge)	// edge 1 -> Rise ; 0 -> fall; (for P pin)
{

	//if (edge == 1){
	//	__HAL_COMP_COMP2_EXTI_DISABLE_FALLING_EDGE() ;
	//	__HAL_COMP_COMP2_EXTI_ENABLE_RISING_EDGE();
	//}
	//else {
	//	__HAL_COMP_COMP2_EXTI_ENABLE_FALLING_EDGE() ;
	//	__HAL_COMP_COMP2_EXTI_DISABLE_RISING_EDGE();
	//}

  hcomp2.Instance = COMP2;
  if ((n==0) || (n==1)) hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
  else if (n==2) 		hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
  else					hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;


#define COMP_CSR_COMPxNONINSEL_MASK      ((uint16_t)COMP_CSR_COMP1SW1)
#define COMP_CSR_COMP2_SHIFT             16U

  uint32_t regshift = COMP_CSR_COMP2_SHIFT;

  MODIFY_REG(COMP->CSR,
             (COMP_CSR_COMPxINSEL  | COMP_CSR_COMPxNONINSEL_MASK | \
              COMP_CSR_COMPxOUTSEL | COMP_CSR_COMPxPOL           | \
              COMP_CSR_COMPxHYST   | COMP_CSR_COMPxMODE) << regshift,
             (hcomp2.Init.InvertingInput    | \
            		 COMP_NONINVERTINGINPUT_IO1 | \
					 COMP_OUTPUT_NONE            | \
					 COMP_OUTPUTPOL_NONINVERTED         | \
					 COMP_HYSTERESIS_HIGH        | \
					 COMP_MODE_HIGHSPEED) << regshift);

}


#define setDriveTable(i,a,b,c,d)   driveTableLow[i].GPIO_Port = a; driveTableLow[i].GPIO_Pin = b; driveTableHight[i].GPIO_Port = c; driveTableHight[i].GPIO_Pin = d



void MakeTables(void)
{
#define INVERSE 0
#if INVERSE
	setDriveTable(5, OUT_A0_GPIO_Port, OUT_A0_Pin, OUT_B1_GPIO_Port, OUT_B1_Pin);
	setDriveTable(4, OUT_A0_GPIO_Port, OUT_A0_Pin, OUT_C1_GPIO_Port, OUT_C1_Pin);
	setDriveTable(3, OUT_B0_GPIO_Port, OUT_B0_Pin, OUT_C1_GPIO_Port, OUT_C1_Pin);
	setDriveTable(2, OUT_B0_GPIO_Port, OUT_B0_Pin, OUT_A1_GPIO_Port, OUT_A1_Pin);
	setDriveTable(1, OUT_C0_GPIO_Port, OUT_C0_Pin, OUT_A1_GPIO_Port, OUT_A1_Pin);
	setDriveTable(0, OUT_C0_GPIO_Port, OUT_C0_Pin, OUT_B1_GPIO_Port, OUT_B1_Pin);


	COMP_CHN_Table[5] = COMP_CHN_C;	//c	rise
	COMP_CHN_Table[4] = COMP_CHN_B;	//b fall
	COMP_CHN_Table[3] = COMP_CHN_A;	//a rise
	COMP_CHN_Table[2] = COMP_CHN_C;	//c fall
	COMP_CHN_Table[1] = COMP_CHN_B;	//b rise
	COMP_CHN_Table[0] = COMP_CHN_A;	//a	fall
#else
	setDriveTable(0, OUT_A0_GPIO_Port, OUT_A0_Pin, OUT_B1_GPIO_Port, OUT_B1_Pin);
	setDriveTable(1, OUT_A0_GPIO_Port, OUT_A0_Pin, OUT_C1_GPIO_Port, OUT_C1_Pin);
	setDriveTable(2, OUT_B0_GPIO_Port, OUT_B0_Pin, OUT_C1_GPIO_Port, OUT_C1_Pin);
	setDriveTable(3, OUT_B0_GPIO_Port, OUT_B0_Pin, OUT_A1_GPIO_Port, OUT_A1_Pin);
	setDriveTable(4, OUT_C0_GPIO_Port, OUT_C0_Pin, OUT_A1_GPIO_Port, OUT_A1_Pin);
	setDriveTable(5, OUT_C0_GPIO_Port, OUT_C0_Pin, OUT_B1_GPIO_Port, OUT_B1_Pin);


	COMP_CHN_Table[0] = COMP_CHN_C;	//c	rise
	COMP_CHN_Table[1] = COMP_CHN_B;	//b fall
	COMP_CHN_Table[2] = COMP_CHN_A;	//a rise
	COMP_CHN_Table[3] = COMP_CHN_C;	//c fall
	COMP_CHN_Table[4] = COMP_CHN_B;	//b rise
	COMP_CHN_Table[5] = COMP_CHN_A;	//a	fall
#endif
}



void SwitchGPIO_OUTPUT(int cStep)
{
	  //HAL_GPIO_WritePin(GPIOA, OUT_A1_Pin|OUT_B1_Pin|OUT_C1_Pin|OUT_A0_Pin, GPIO_PIN_RESET);
	  //HAL_GPIO_WritePin(GPIOB, OUT_B0_Pin|OUT_C0_Pin, GPIO_PIN_RESET);

	GPIOA->BRR = OUT_A1_Pin|OUT_B1_Pin|OUT_C1_Pin|OUT_A0_Pin;
	GPIOB->BRR = OUT_B0_Pin|OUT_C0_Pin;


	driveTableHight[cStep].GPIO_Port->BSRR = driveTableHight[cStep].GPIO_Pin;

	  //HAL_GPIO_WritePin(driveTableHight[cStep].GPIO_Port, driveTableHight[cStep].GPIO_Pin, GPIO_PIN_SET);
	  //HAL_GPIO_WritePin(driveTableLow[cStep].GPIO_Port, driveTableLow[cStep].GPIO_Pin, GPIO_PIN_SET);

}


int signal_on = 0;

void SwitchStep(void)
{
	CounterTIM2Read=0;
	ChangeTime = 0;

    if (signal_on == 1)driveTableLow[nextCommutationStep].GPIO_Port->BRR = driveTableLow[nextCommutationStep].GPIO_Pin;		//Reset Last Communication FET
    if (++nextCommutationStep >= 6) nextCommutationStep = 0;

    //DRIVE_PORT = driveTable[nextCommutationStep];
    SwitchGPIO_OUTPUT(nextCommutationStep);
    if (signal_on == 1)driveTableLow[nextCommutationStep].GPIO_Port->BSRR = driveTableLow[nextCommutationStep].GPIO_Pin;	//Set Communication FET

    zcPolarity = (nextCommutationStep) & 0x01;
    MX_COMP2_Init_Channel(COMP_CHN_Table[nextCommutationStep], zcPolarity);//ADMUX = ADMUXTable[nextCommutationStep];		//SET Compare to communication step

}


int ReaderNextStep(void)
{
	if (CounterTIM2Read<filteredTimeSinceCommutation*2) return 0;

	if(1)if (!ChangeTime){
        if (filteredTimeSinceCommutation<STURTUP_TIME) filteredTimeSinceCommutation += 2*(filteredTimeSinceCommutation/(COMMUTATION_TIMING_IIR_COEFF_B));
        incorrectfl++;
    }

	SwitchStep();
	return 1;
}



void COMP_Reader(void)
{
	if (startup_time < 500) {											//Startup rotate motor. Without compare.
		filteredTimeSinceCommutation = 3000 - (startup_time*2);
		return;
	}

	if (ChangeTime==1) return;

	unsigned int timer2 = CounterTIM2Read;
	//if (timer2<(TIME_PERIOD/2)) return;
	//if (timer2<(35)) return;	//35 was

	if ((TimeRotate>=5)&& (timer2<(10))) return;
	else if (timer2<(35)) return;	//35 was

	//unsigned int timer1 = CounterTIM1Read;
	//if (timer1<5) return;

	if ((READ_BIT(COMP->CSR, COMP_CSR_COMPxOUT << COMP_CSR_COMP2_SHIFT)==0) == (zcPolarity==0)){

		//timeSinceCommutation_Value = timer2;
		if ((ChangeTime==0)){
			ChangeTime=1;
			filteredTimeSinceCommutation = (COMMUTATION_TIMING_IIR_COEFF_A * (timer2*1)
									+ COMMUTATION_TIMING_IIR_COEFF_B * filteredTimeSinceCommutation)
									/ (COMMUTATION_TIMING_IIR_COEFF_A + COMMUTATION_TIMING_IIR_COEFF_B);
			if (filteredTimeSinceCommutation>STURTUP_TIME) filteredTimeSinceCommutation=STURTUP_TIME;

			correctfl++;
		}
	}
}

//int ttt=0;
void whileImpulse(void)
{
	int i=0;

	CounterTIM6Read=0;
	driveTableLow[nextCommutationStep].GPIO_Port->BSRR = driveTableLow[nextCommutationStep].GPIO_Pin;
	CounterTIM1Read=0;
	signal_on=1;
	//uDelay(TIME_IMPULS);
	while(CounterTIM6Read<TIME_IMPULS){COMP_Reader(); if (ReaderNextStep()) return;};

	for (i=0;i<6;i++)driveTableLow[i].GPIO_Port->BRR = driveTableLow[i].GPIO_Pin;
	CounterTIM1Read=0;
	signal_on=0;

	//uDelay(10);//uDelay(2);//uDelay(50);
	//while(CounterTIM6Read<(TIME_IMPULS+5)){ReaderNextStep();};

	while(CounterTIM6Read<TIME_PERIOD){   //while(CounterTIM1Read<(delay-TIME_IMPULS+timerCNT)){

		COMP_Reader();

		if (ReaderNextStep()) return;

	}



}



void SettingsToDefault(void){

	MakeTables();

	__HAL_COMP_COMP2_EXTI_ENABLE_RISING_EDGE();
	__HAL_COMP_COMP2_EXTI_ENABLE_FALLING_EDGE();

	filteredTimeSinceCommutation = 0;
	nextCommutationStep = 0;
	filteredTimeSinceCommutation = STURTUP_TIME;

	HAL_GPIO_WritePin(GPIOA, OUT_A1_Pin|OUT_B1_Pin|OUT_C1_Pin|OUT_A0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, OUT_B0_Pin|OUT_C0_Pin, GPIO_PIN_RESET);

}




int sTimer=0;

void HAL_SYSTICK_Callback(void)
{

	if (TIME_IMPULS!=0){if(startup_time<3000)startup_time++;}
	else startup_time=0;
	if (incorrectfl >200) {TIME_IMPULS=0; sTimer=0; incorrectfl=0; return;}


/*	if (++TimerErrorControl>50){
		if ((2*incorrectfl) > correctfl){TIME_IMPULS=0; sTimer=0;}
		TimerErrorControl=0;
		incorrectfl=0;
		correctfl=0;
	}*/
//	if (++sTimer>=1000) {sTimer=0; Printf("timeSinceCommutation_Value = %d\n\r", timeSinceCommutation_Value); }
//if (++sTimer>=1000) {sTimer=0; Printf("%d us\n\r", filteredTimeSinceCommutation*12);}//timeSinceCommutation_Value
//if (++sTimer>=1000) {sTimer=0; Printf("%d us, %d us\n\r", filteredTimeSinceCommutation*12, Impulse_Long);}//timeSinceCommutation_Value

	if (++sTimer>=1000) {
		//sTimer=0; Printf("%d us, %d us, incorr=%d\n\r", filteredTimeSinceCommutation*12, Impulse_Length, incorrectfl);
		//sTimer=0; Printf("%d us, incorr=%d\n\r", filteredTimeSinceCommutation*12, incorrectfl);
		sTimer=0; Printf("%d us\n\r", filteredTimeSinceCommutation*12);
		//sTimer=0; Printf("%d us, imp:%d\n\r", filteredTimeSinceCommutation, TIME_IMPULS);
		incorrectfl=0;
		//if (Impulse_Ready !=0){TIME_IMPULS = Impulse_Length; Impulse_Ready=0;}
		//else TIME_IMPULS=0;
		if (Impulse_Ready == 0)TIME_IMPULS=0;
		//TIME_IMPULS = Impulse_Long;
		//if (++sTimer>=1000) {sTimer=0; Printf("%d\n\r", TIME_IMPULS);}//timeSinceCommutation_Value


		if (TIME_IMPULS>0){TimeRotate++;}
		else TimeRotate=0;
	}

	if (sTimer==500) {
		//Printf("imp:%d, ", TIME_IMPULS);
	}

}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_COMP2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  //HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim6);

  HAL_COMP_MspInit_Add(&hcomp2);

  if(HAL_COMP_Start_IT(&hcomp2) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }

  MX_COMP2_Init_Channel(0, 0);


  SettingsToDefault();


  Impulse_Ready=0;
  while((Impulse_Ready == 0) && (Impulse_Length != 0)){HAL_Delay(1);}

  Printf("Started\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	  while(TIME_IMPULS<10){ SettingsToDefault(); HAL_Delay(10);}

	  whileImpulse();



  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* COMP2 init function */
static void MX_COMP2_Init(void)
{

  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp2.Init.Output = COMP_OUTPUT_NONE;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_HIGH;
  hcomp2.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 48;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xFFFF;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT_A1_Pin|OUT_B1_Pin|OUT_C1_Pin|OUT_A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_B0_Pin|OUT_C0_Pin|TEST_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OUT_A1_Pin OUT_B1_Pin OUT_C1_Pin OUT_A0_Pin */
  GPIO_InitStruct.Pin = OUT_A1_Pin|OUT_B1_Pin|OUT_C1_Pin|OUT_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_B0_Pin OUT_C0_Pin TEST_OUT_Pin */
  GPIO_InitStruct.Pin = OUT_B0_Pin|OUT_C0_Pin|TEST_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_IN_Pin */
  GPIO_InitStruct.Pin = TEST_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TEST_IN_GPIO_Port, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_CFGR1_I2C_FMP_PB7);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
