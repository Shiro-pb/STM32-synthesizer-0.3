/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "Chip.h"
//#include "stm32f103x6.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define bufferSamplesLowThreshold 100
#define bufferSamplesHighThreshold 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

ADC_HandleTypeDef g_AdcHandle;

uint16_t outputSum;

MIDIChannel volatile channels[3] = {0};

SynthState synthState = {0};

uint8_t incommingMidiCommand[3] = {0};

int currentChannel = 0;

double squarePhase;

//buffer stuff
bool bufferNeedsSamples = true;
unsigned int sampleNumberInBuffer = 0;
unsigned int sampleNumberInBuffer2 = 0;
unsigned int bufferAddIndex = 0;
unsigned int bufferSubtractIndex = 0;

uint16_t sampleBuffer[1640]; // 0 to 1639

//arpeggio stuff
int loopPlayingChannel = 0;
int playingChannel = 0;

int arpCount = 0;

//EventMatrix sequencerMatrix [8] [3];//8 as in (1 & 2 & 3 & 4 &)   ;   3 channels

//double noteTable[] = {32.703, 34.648, 36.708, 38.891, 41.203, 43.654, 46.249, 48.999, 41.913, 55, 58.270, 61.735, 65.406, 69.296, 73.416, 77.782, 82.407, 87.307, 92.499, 97.999, 103.83, 110.0, 116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.0, 220.0, 207.65, 246.94,};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t synthCompute(void){
  uint16_t outputSum = 0;
  if(synthState.arpeggioOn){
    outputSum = midiChannelOutputCompute(&channels[playingChannel], synthState);
    if(arpCount == 0){
      loopPlayingChannel = playingChannel + 1;
      while(1){
      if(channels[loopPlayingChannel].gate == 1){      //next channel found
        playingChannel = loopPlayingChannel;
        break;
      }
      if(loopPlayingChannel == playingChannel){       // all channels 0
        //something to let the computer know that no channel is playing
        break;
      }
      loopPlayingChannel++;
      if(loopPlayingChannel > channelNumber) loopPlayingChannel = 0;
      }
    }
    arpCount++;
    if(arpCount > synthState.arpSpeed){
      arpCount = 0;
    }
  } else {
    outputSum = midiChannelOutputCompute(&channels[0], synthState);
  }

  return outputSum;
}

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ADC_Init();
  chipophoneInit();

  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if (HAL_UART_Receive(&huart2, incommingMidiCommand, 3, 10) == HAL_OK){//slow
      huart2.Instance->DR = 0;
      switch (incommingMidiCommand[0] & 0b11110000){
        case 0x90:
        if(synthState.arpeggioOn){
          channels[currentChannel].noteValue = incommingMidiCommand[1];
          channels[currentChannel].noteVelocity = incommingMidiCommand[2];
          channels[currentChannel].gate = true;
          currentChannel++;
          if(currentChannel > 3) currentChannel = 0;
        } else {
          channels[0].noteValue = incommingMidiCommand[1];
          channels[0].noteVelocity = incommingMidiCommand[2];
          channels[0].gate = true;
        }
        /*
          switch (currentChannel)
          {
          case 1:
            channel1.noteValue = incommingMidiCommand[1];
            channel1.noteVelocity = incommingMidiCommand[2];
            channel1.gate = true;
            break;
          
          case 2:
            channel2.noteValue = incommingMidiCommand[1];
            channel2.noteVelocity = incommingMidiCommand[2];
            channel2.gate = true;
            break;
          
          case 3:
            channel3.noteValue = incommingMidiCommand[1];
            channel3.noteVelocity = incommingMidiCommand[2];
            channel3.gate = true;
            break;

          case 4:
            channel4.noteValue = incommingMidiCommand[1];
            channel4.noteVelocity = incommingMidiCommand[2];
            channel4.gate = true;
            break;

          case 5:
            channel5.noteValue = incommingMidiCommand[1];
            channel5.noteVelocity = incommingMidiCommand[2];
            channel5.gate = true;
            break;

          case 6:
            channel6.noteValue = incommingMidiCommand[1];
            channel6.noteVelocity = incommingMidiCommand[2];
            channel6.gate = true;
            break;

          default:
            break;
          }
          
          
          currentChannel++;
          if(currentChannel > 2){
            currentChannel = 1;
          }
          */
          break;
          

        case 0x80:


          //if(channel1.noteValue == incommingMidiCommand[1]){
          //  channels[0].noteValue = 0x00;
          //  channels[0].noteVelocity = 0x00;
           // channels[0].gate = false;
          //}
          
          for(int i = 0 ; i < channelNumber ; i++){
            if(channels[i].noteValue == incommingMidiCommand[1]){
              channels[i].noteValue = 0x00;
              channels[i].noteVelocity = 0x00;
              channels[i].gate = false;
            }
          }
          /*
          if(channel2.noteValue == incommingMidiCommand[1]){
            //channel2.noteVelocity = 0x00;
            channel2.gate = false;
          }
          if(channel3.noteValue == incommingMidiCommand[1]){
            //channel3.noteVelocity = 0x00;
            channel3.gate = false;
          }
          if(channel4.noteValue == incommingMidiCommand[1]){
            //channel4.noteVelocity = 0x00;
            channel4.gate = false;
          }
          if(channel5.noteValue == incommingMidiCommand[1]){
            //channel5.noteVelocity = 0x00;
            channel5.gate = false;
          }
          if(channel6.noteValue == incommingMidiCommand[1]){
            //channel6.noteVelocity = 0x00;
            channel6.gate = false;
          }
         */

          break;
      }
    }

    //buffer handling
    if(bufferSubtractIndex > 1639){
      bufferSubtractIndex = 0;
    }
    if(bufferAddIndex > 1639){
      bufferAddIndex = 0;
    }
    if(sampleNumberInBuffer2 <= 10){
      bufferNeedsSamples = true;
    }
    if(sampleNumberInBuffer2 >= 1500){
      bufferNeedsSamples = false;
    }
    if(bufferNeedsSamples){
      sampleBuffer[bufferAddIndex] = synthCompute();
      sampleNumberInBuffer++;
      sampleNumberInBuffer2++;
      bufferAddIndex++;
    }

    synthAtt(&synthState);
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0x0B26 ;//0x066a / 0x960 for 30k /b40 f0r 25k (b26 for tune)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 31250;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 
                           PB12 PB13 PB14 PB15 
                           PB4 PB5 PB6 PB7 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ADC_Init(void){
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;//clock divider (/6)
  
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;//clock enable

  //configure pin as input push pull
  GPIOA->CRL |= GPIO_CRL_CNF0_1;
  GPIOA->CRL &= ~(GPIO_CRL_CNF0_0);
    
  
  ADC1->CR1 = 0x00;//resset both CR1 and CR2 registers
  ADC1->CR2 = 0x00;

  ADC1->SMPR2 = ADC_SMPR2_SMP0;

  //ADC1->CR2 |= ADC_CR2_ALIGN;// left data alignment

  ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT;//turn adc on

  HAL_Delay(1);

  ADC1->CR2 |= ADC_CR2_ADON;

  HAL_Delay(1);

  ADC1->CR2 |= ADC_CR2_CAL;

  HAL_Delay(1);
}


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
