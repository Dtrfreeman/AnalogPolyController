/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */


volatile uint8_t DacData[8];
volatile uint8_t SetNoteCounter=0;//counts number of stages complete
void setDacBufVal(uint8_t channel,uint16_t * dataIn){
	channel=channel%4;
	DacData[channel<<1]=((*dataIn)>>8)&0x0f;
	DacData[(channel<<1)+1]=(*dataIn)&0xff;
}
	
void writeToDac(){
	HAL_I2C_Master_Transmit(&hi2c2,0x00c2,(uint8_t *)&DacData[0],0x0008,1);
	
	//frustratingly this is blocking.
}


struct voice{
	uint8_t noteOn;
	uint8_t noteCode;
	uint8_t lState;
	uint8_t fState;
	uint8_t noteVel;
	uint16_t lEnvCnt;
	uint16_t fEnvCnt;
	volatile unsigned int * loudnessChannel;
	volatile unsigned int * filterChannel;
};

uint8_t midiBuf,noteBuf,velBuf,curPos,status=0;
volatile struct voice VoiceArray[4];

void initVoices(){
	
	VoiceArray[0].loudnessChannel=(&(TIM4->CCR1));
	VoiceArray[0].filterChannel=(&(TIM3->CCR1));
	
	VoiceArray[1].loudnessChannel=(&(TIM4->CCR2));
	VoiceArray[1].filterChannel=(&(TIM3->CCR2));
	
	VoiceArray[2].loudnessChannel=(&(TIM4->CCR3));
	VoiceArray[2].filterChannel=(&(TIM2->CCR1));
	
	VoiceArray[3].loudnessChannel=(&(TIM4->CCR4));
	VoiceArray[3].filterChannel=(&(TIM2->CCR2));
	for(uint8_t v=0;v<4;v++){
		VoiceArray[v].noteOn=0;
		VoiceArray[v].lState=0;
		VoiceArray[v].fState=0;
		VoiceArray[v].noteCode=0;
		VoiceArray[v].noteVel=0;
		VoiceArray[v].lEnvCnt=0;
		VoiceArray[v].fEnvCnt=0;
		*VoiceArray[v].loudnessChannel=0;
		*VoiceArray[v].filterChannel=0;
	}
}
double fError[4]={0,0,0,0};
double eError[4]={0,0,0,0};
/*
f error is offset at zero
e error is bits per octave error 
*/
void noteCodeToDac(uint8_t curVoice){
	
	uint16_t dataBuf=round(log2((8.17525*VoiceArray[curVoice].noteCode)/(32.7+fError[curVoice]))/(683+eError[curVoice]));
	setDacBufVal(curVoice,&dataBuf);
	return;
}

void setNote(uint8_t note){
	uint8_t curVoice=0;
	while(VoiceArray[curVoice].noteOn!=0){curVoice++;
	if(curVoice==4){return;}}
	SetNoteCounter=1;
	VoiceArray[curVoice].noteOn=1;
	VoiceArray[curVoice].lState=1;
	VoiceArray[curVoice].fState=1;
	VoiceArray[curVoice].noteCode=note;
	VoiceArray[curVoice].lEnvCnt=0;
	VoiceArray[curVoice].fEnvCnt=0;
	VoiceArray[curVoice].noteVel=velBuf;
	writeToDac();
	return;
}

void releaseNote(uint8_t note){
	uint8_t curVoice=0;
	while(VoiceArray[curVoice].noteCode!=note){
		curVoice++;
		if(curVoice==4){return;}}
	VoiceArray[curVoice].noteOn=0;
	VoiceArray[curVoice].lState=5;
	VoiceArray[curVoice].fState=5;
	return;
}

void midiParse(){
	switch(curPos){
			case 0:{if(midiBuf>=0x80){
					status=midiBuf;
					status&=0xf0;
					if(status==0x90||status==0x80){
						HAL_UART_DMAStop(&huart1);
						HAL_UART_Receive_DMA(&huart1,&noteBuf,1);
					
						curPos=1;}
					}
					else{
						
					if(status==0x90||status==0x80){
						noteBuf=midiBuf;
						HAL_UART_DMAStop(&huart1);
						HAL_UART_Receive_DMA(&huart1,&velBuf,1);
						curPos=2;}
					}
			break;
			}
			case 1:{
				HAL_UART_DMAStop(&huart1);
					HAL_UART_Receive_DMA(&huart1,&velBuf,1);
					curPos=2;
			break;
			}
			
			case 2:{
				curPos=0;
				HAL_UART_DMAStop(&huart1);
				HAL_UART_Receive_DMA(&huart1,&midiBuf,1);
				switch(status){
					case 0x90:{
						setNote(noteBuf);
						break;}
					case 0x80:{
						releaseNote(noteBuf);
						break;}}
			break;
			}
}}

void DMA1_Channel5_IRQHandler(void)
{
	
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
	midiParse();
  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

void USART1_IRQHandler(void){
	midiParse();
		HAL_UART_IRQHandler(&huart1);
		return;
}
volatile uint8_t EnvSampleFlag=0;

void timer1Complete(){
	EnvSampleFlag=1;

}

void incrementEnvVal(uint8_t curVoice,signed int Change,uint16_t upperLim,uint16_t lowerLim){
	
	
	volatile unsigned int * Channel;
	if(curVoice<4){Channel=VoiceArray[curVoice].loudnessChannel;}
	else if(curVoice<8){Channel=VoiceArray[curVoice-4].filterChannel;}
	else{return;}
	
	uint16_t curVal=*(Channel);
	uint16_t valueToBe=curVal+Change;
	if(valueToBe<lowerLim){
		if(curVal==lowerLim){return;}
		else{
			valueToBe=lowerLim;
		}
	}
	else if(valueToBe>upperLim){
		if(curVal==upperLim){return;}
		else{
			valueToBe=upperLim;
		}
	}
	*(Channel)=valueToBe/4;
	
	
	return;	
}

uint8_t attackToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit){//returns 0 once the output reaches value
	volatile unsigned int * Channel;
	if(curVoice<4){Channel=VoiceArray[curVoice].loudnessChannel;}
	else if(curVoice<8){Channel=VoiceArray[curVoice-4].filterChannel;}
	else{return(0);}
	uint16_t curChannelVal=*Channel;
	if(curChannelVal==limit){return(0);}//if at limit itll exit with a state complete code
	
	
	else if((curChannelVal+gradient)<=limit){*Channel=curChannelVal+gradient;return(1);}//increases by gradient and exits with state continue code
	
	else if(curChannelVal>limit){//if for whatever reason its more than limit itll exit with a state complete code
		curChannelVal=limit;
		return(0);}
	
		
	else if((curChannelVal+gradient)>limit){//if once the gradient is added itll exceed the limit itll
		*Channel=limit;
		return(0);
	}
	
	
	return(0);
}
uint8_t releaseToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit){//returns 0 once the output reaches limit
	
	volatile unsigned int * Channel;
	if(curVoice<4){Channel=VoiceArray[curVoice].loudnessChannel;}
	else if(curVoice<8){Channel=VoiceArray[curVoice-4].filterChannel;}
	else{return(0);}
	uint16_t curChannelVal=*Channel;
	if(curChannelVal==limit){return(0);}//if at limit itll exit with a state complete code
	
	
	else if((curChannelVal-gradient)>=limit){*Channel=curChannelVal-gradient;return(1);}//decreases by gradient and exits with state continue code
	
	else if(curChannelVal<limit){//if for whatever reason its more than limit itll exit with a state complete code
		curChannelVal=limit;
		return(0);}
	
		
	else if((curChannelVal+gradient)<limit){//if once the gradient is added itll go under the limit itll set the output to the limit and return a state complete code
		*Channel=limit;
		return(0);
	}
	
	
	return(0);
	
}
/*	    __
			 /  \
      /    \
		 /      \_________
	  /                 \
	 /                   \
  /                     \   
-><----><><-><-------><-><----
0	   1  2  3     4     5  0

ADSRvals formatted 0:lAttack,1:lDelay,2:lSustain,3:lRelease,4:fAttack,5:fDelay,6:fSustain,7:fRelease,8:fInfluence
*/


void lADSRstep(uint8_t voiceNum,uint16_t * pADSRvals ){
	switch(VoiceArray[voiceNum].lState){
		case 0:return;
		case 1:{//attack
			if(VoiceArray[voiceNum].lEnvCnt< *(pADSRvals+1)){
				if(attackToVal(voiceNum,*(pADSRvals),1024)==0){
					VoiceArray[voiceNum].lState=2;
				}
				VoiceArray[voiceNum].lEnvCnt++;
			}
			else{VoiceArray[voiceNum].lState=3;}
			return;
		}
		case 2:{//once attack has reached peak value
			if(VoiceArray[voiceNum].lEnvCnt>= *(pADSRvals+1)){
				VoiceArray[voiceNum].lState=3;
			}
			else{VoiceArray[voiceNum].lEnvCnt++;}
			return;
		}
		case 3:{//releasing to sustain
			if(releaseToVal(voiceNum,*(pADSRvals+3),*(pADSRvals+2))==0){VoiceArray[voiceNum].lState=4;}
			return;
		}
		case 4:return;
		case 5:{//releasing to 0
			if(releaseToVal(voiceNum,*(pADSRvals+3),0)==0){VoiceArray[voiceNum].lState=0;}
			return;
		return;}
		
		
	}


}
void fADSRstep(uint8_t voiceNum,uint16_t * pADSRvals ){
	switch(VoiceArray[voiceNum].fState){
		case 0:return;
		case 1:{//attack
			if(VoiceArray[voiceNum].fEnvCnt< *(pADSRvals+5)){
				if(attackToVal(voiceNum,*(pADSRvals+4),*(pADSRvals+8)/4)==0){
					VoiceArray[voiceNum].fState=2;
				}
				VoiceArray[voiceNum].fEnvCnt++;
			}
			else{VoiceArray[voiceNum].fState=3;}
			return;
		}
		case 2:{//once attack has reached peak value
			if(VoiceArray[voiceNum].fEnvCnt>= *(pADSRvals+5)){
				VoiceArray[voiceNum].fState=3;
			}
			else{VoiceArray[voiceNum].fEnvCnt++;}
			return;
		}
		case 3:{//refeasing to sustain
			if(releaseToVal(voiceNum,*(pADSRvals+7),*(pADSRvals+6))==0){VoiceArray[voiceNum].fState=4;}
			return;
		}
		case 4:return;
		case 5:{//refeasing to 0
			if(releaseToVal(voiceNum,*(pADSRvals+7),0)==0){VoiceArray[voiceNum].fState=0;}
			return;
		return;}
		
		
	}

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_DMA(&huart1,&midiBuf,1);
	HAL_DMA_Init(&hdma_usart1_rx);
	
	HAL_TIM_OC_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim1);
	
	uint16_t ADSRvals[9]={180,2048,2048,180,180,2048,2048,180,2048};//lAttack,lDelay,lSustain,lRelease,fAttack,fDelay,fSustain,fRelease,fInfluence
	uint16_t ADSRBuf[9]={180,2048,2048,180,180,2048,2048,180,2048};//lAttack,lDelay,lSustain,lRelease,fAttack,fDelay,fSustain,fRelease,fInfluence
	initVoices();
	
	//*VoiceArray[0].loudnessChannel=256;for test purposes
	//*VoiceArray[0].filterChannel=256;
	

	uint8_t channel=0;
	HAL_DMA_Init(&hdma_i2c2_tx);
	HAL_I2C_Init(&hi2c2);
	for(uint8_t i=0;i<8;i++){
		HAL_I2C_IsDeviceReady(&hi2c2,0x00c0|i<<1,1,2);
	
	}
	
	uint8_t rescanCnt=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		
		if(channel<4){
			lADSRstep(channel,&ADSRvals[0]);
			fADSRstep(channel,&ADSRvals[0]);
			channel++;
		}
		else if(EnvSampleFlag==1){
			channel=0;
			rescanCnt++;
			if(rescanCnt>=10){
				

				HAL_ADC_Stop_DMA(&hadc1);
				HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&ADSRBuf[0],18);
			}
			if(rescanCnt>=11){
					rescanCnt=0;
					for(signed int i=8;i>=0;i--){
						//implements rough joystick curve
						if(ADSRBuf[i]<1024){
							ADSRBuf[i]=ADSRBuf[i]*1.7;}
						else if(ADSRBuf[i]<3072){
								ADSRBuf[i]=(ADSRBuf[i]/3)+1400;
						}
						else{
							ADSRBuf[i]=(ADSRBuf[i]*1.7)-2867;
						}
						switch(i){//scales sustain down by 4 and attack and release by 4 and scales them with the influence on the filter
							case 0:{ADSRvals[i]=ADSRBuf[i]/16;
								break;}
							case 2:{ADSRvals[i]=ADSRBuf[i]/4;
								break;}
							case 3:{ADSRvals[i]=ADSRBuf[i]/16;
								break;}
							case 4:{ADSRvals[i]=(uint16_t)(((unsigned long)ADSRBuf[i]*ADSRBuf[8])/65536);
								break;}
							case 6:{ADSRvals[i]=(uint16_t)(((unsigned long)ADSRBuf[i]*ADSRBuf[8])/16383);;
								break;}
							case 7:{ADSRvals[i]=(uint16_t)(((unsigned long)ADSRBuf[i]*ADSRBuf[8])/65536);
								break;}
							default:{ADSRvals[i]=ADSRBuf[i];break;}
						}
						
					}
					
					printf("ADSR Values are A%u D%u S%u R%u filter: A%u D%u S%u R%u with influence %u",ADSRvals[0],ADSRvals[1],ADSRvals[2],ADSRvals[3],ADSRvals[4],ADSRvals[5],ADSRvals[6],ADSRvals[7],ADSRvals[8]);//you made it
			}
		}
	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_EXT_IT11;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 36;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1024;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1024;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1024;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
