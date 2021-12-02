/**
 * @file main.c
 * @brief Node main source code file
 * @version 0.1
 * @date 2021-11-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <stdio.h>
#include <stm32l4xx_hal.h>
#include <string.h>

void USART1_Init(void);
void USART2_Init(void);
void GPIO_init(void);
void ADC_Init(void);
void SystemClock_Config(void);
void LED_Init(void);

void Read_RPM(void);

void sanitize_ADC(void);
void sanitize_RPM(void);

static UART_HandleTypeDef h_UARTHandle;
static ADC_HandleTypeDef h_ACDC1Handle;

uint32_t ADC_Value = 0;
char ADC_String[5] = "0";
char RPM_String[5] = "0";
char Data_String[11] = "";
uint32_t tickstorage;
int RPM=0;

uint32_t tick_count;
uint32_t elapsed_time;

int count=0;
int main(void)
{   
  __disable_irq();
  SystemClock_Config();
  HAL_Init();
  SystemCoreClockUpdate();

  USART1_Init();
  ADC_Init();

  GPIO_init();
  LED_Init();

  HAL_GPIO_WritePin(GPIOA, 8, GPIO_PIN_SET); //Set MAX3485 transmit enable signal
  //NVIC_EnableIRQ(USART2_IRQn); //Enable USART interrupt handeler
  __enable_irq();
  
  tickstorage = HAL_GetTick(); //Get start tick count for first RPM calcluation

  while(1)
  {
    HAL_ADC_Start(&h_ACDC1Handle);
    HAL_Delay(4);

    if(HAL_ADC_PollForConversion(&h_ACDC1Handle, 10) == HAL_OK) //Check that conversion completes
    {
      ADC_Value = HAL_ADC_GetValue(&h_ACDC1Handle); //Store ADC value
    }
    HAL_ADC_Stop(&h_ACDC1Handle); // stop conversion 
    
    sanitize_ADC();
    Read_RPM();
    sanitize_RPM();

    sprintf(Data_String, "T%sP%s", ADC_String, RPM_String);

    if (count == 10)
    {
      HAL_UART_Transmit(&h_UARTHandle, (uint8_t *)Data_String, strlen(Data_String), HAL_MAX_DELAY);
      count=0;
    }
    else{
      count++;
    }


    HAL_Delay(10);
  }

  return 0;
}

void Read_RPM(void)
{
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
    {
      tick_count = HAL_GetTick();

      elapsed_time = tick_count - tickstorage;

      RPM = 60000/elapsed_time; //Events per second times 60 to calculate RPM

      tickstorage = tick_count; //Reset event tick storage for next calculation

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
      if (RPM > 0)
      {
        RPM--;
      }
      
    }

}

void sanitize_ADC(void)
{
  strcpy(ADC_String, "");
  if (ADC_Value >= 0 && ADC_Value < 10)
  {
    sprintf(ADC_String,"000%d",ADC_Value);
  }
  else if (ADC_Value >= 10 && ADC_Value < 100)
  {
    sprintf(ADC_String,"00%d",ADC_Value);
  }
  else if (ADC_Value >= 100 && ADC_Value < 1000)
  {
    sprintf(ADC_String,"0%d",ADC_Value);
  }
  else if (ADC_Value >= 1000)
  {
    sprintf(ADC_String,"%d",ADC_Value);
  }

}

void sanitize_RPM(void)
{
  strcpy(RPM_String, "");
  if (RPM >= 0 && RPM < 10)
  {
    sprintf(RPM_String,"000%d",RPM);
  }
  else if (RPM >= 10 && RPM < 100)
  {
    sprintf(RPM_String,"00%d",RPM);
  }
  else if (RPM >= 100 && RPM < 1000)
  {
    sprintf(RPM_String,"0%d",RPM);
  }
  else if (RPM >= 1000)
  {
    sprintf(RPM_String,"%d",RPM);
  }

}

void USART2_IRQHandler(void)
{
  __disable_irq();

  HAL_UART_Transmit(&h_UARTHandle, (uint8_t *)"TEST", strlen("TEST"), HAL_MAX_DELAY);
  HAL_UART_Transmit(&h_UARTHandle, (uint8_t *)"\n", sizeof(char), HAL_MAX_DELAY);
  HAL_UART_Transmit(&h_UARTHandle, (uint8_t *)"\r", sizeof(char), HAL_MAX_DELAY);

  __HAL_UART_SEND_REQ(&h_UARTHandle, UART_RXDATA_FLUSH_REQUEST);
  __HAL_UART_CLEAR_IT(&h_UARTHandle,UART_CLEAR_OREF);
  __enable_irq();

}

void USART1_IRQHandler(void)
{
  __disable_irq();
  HAL_GPIO_WritePin(GPIOA, 8, GPIO_PIN_SET); //Set MAX3485 transmit enable signal

  HAL_Delay(10);
  HAL_UART_Transmit(&h_UARTHandle, (uint8_t *)ADC_String, sizeof(ADC_String), HAL_MAX_DELAY);

  __HAL_UART_SEND_REQ(&h_UARTHandle, UART_RXDATA_FLUSH_REQUEST); //Flush RX buffers and whatsnots
  __HAL_UART_CLEAR_IT(&h_UARTHandle,UART_CLEAR_OREF);

   HAL_GPIO_WritePin(GPIOA, 8, GPIO_PIN_RESET); //Set MAX3485 receive enable signal
  __enable_irq();

}

void USART1_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_9; //PA9 TX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate =  GPIO_AF7_USART1; 
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
  
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_10; //PA10 RX 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  h_UARTHandle.Instance        = USART1;
  h_UARTHandle.Init.BaudRate   = 9600;
  h_UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
  h_UARTHandle.Init.StopBits   = UART_STOPBITS_1;
  h_UARTHandle.Init.Parity     = UART_PARITY_NONE;
  h_UARTHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  h_UARTHandle.Init.Mode       = UART_MODE_TX_RX;

  HAL_UART_Init(&h_UARTHandle);

  //USART1->CR1 |= USART_CR1_RXNEIE; //Enable RX interrupt

}

void USART2_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_2; //PA2 TX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate =  GPIO_AF7_USART2; 
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
  
    GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
    GPIO_InitStruct.Pin = GPIO_PIN_15; //PA15 RX 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  h_UARTHandle.Instance        = USART2;
  h_UARTHandle.Init.BaudRate   = 115200;
  h_UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
  h_UARTHandle.Init.StopBits   = UART_STOPBITS_1;
  h_UARTHandle.Init.Parity     = UART_PARITY_NONE;
  h_UARTHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  h_UARTHandle.Init.Mode       = UART_MODE_TX_RX;

  HAL_UART_Init(&h_UARTHandle);

  //USART1->CR2 |= USART_CR1_RXNEIE; //Enable RX interrupt

}

void ADC_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC_CLK_ENABLE();

  //INIT ADC PIN
  GPIO_InitTypeDef ADCpin; //create an instance of GPIO_InitTypeDef C struct
  ADCpin.Pin = GPIO_PIN_3; // Select pin PA3/A2
  ADCpin.Mode = GPIO_MODE_ANALOG; // Select Analog Mode
  ADCpin.Pull = GPIO_NOPULL; // Disable internal pull-up or pull-down resistor
  HAL_GPIO_Init(GPIOA, &ADCpin); // initialize PA3 as analog input pin

  //INIT ADC 
  h_ACDC1Handle.Instance = ADC1; // create an instance of ADC1
  h_ACDC1Handle.Init.Resolution = ADC_RESOLUTION_12B; // select 12-bit resolution
  h_ACDC1Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //select  single conversion as a end of conversion event
  h_ACDC1Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT; // set digital output data right justified
  h_ACDC1Handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  
  h_ACDC1Handle.Init.ContinuousConvMode = DISABLE;
  h_ACDC1Handle.Init.ScanConvMode = DISABLE;
  
  h_ACDC1Handle.Init.NbrOfConversion = 1;
  h_ACDC1Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  h_ACDC1Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  h_ACDC1Handle.Init.DMAContinuousRequests = DISABLE;
  h_ACDC1Handle.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  h_ACDC1Handle.Init.OversamplingMode = DISABLE;
  HAL_ADC_Init(&h_ACDC1Handle);

  //INIT ADC CHANNEL
  ADC_ChannelConfTypeDef Channel_AN; // create an instance of ADC_ChannelConfTypeDef
  Channel_AN.Channel = ADC_CHANNEL_8; // select analog channel 8 (ADC1_IN8)
  Channel_AN.Rank = 1; // set rank to 1
  Channel_AN.SamplingTime = ADC_SAMPLETIME_2CYCLES_5; // set sampling time to 15 clock cycles
  Channel_AN.OffsetNumber = ADC_OFFSET_NONE;
  Channel_AN.SingleDiff = ADC_SINGLE_ENDED; //Single ended mode possibly changes the ADC reference points to Ground and ADD?
  Channel_AN.Offset = 0;
  HAL_ADC_ConfigChannel(&h_ACDC1Handle, &Channel_AN); // select channel_0 for ADC2 module.
  
}

void GPIO_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_ADC_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  //PA8 set as a digital output to transmit MAX3485 enable signal
  GPIO_InitTypeDef GPIOPin; //create an instance of GPIO_InitTypeDef C struct
  GPIOPin.Pin = GPIO_PIN_8; // Select pin PA8
  GPIOPin.Mode = GPIO_MODE_OUTPUT_PP; // Select Digital output
  GPIOPin.Pull = GPIO_NOPULL; // Disable internal pull-up or pull-down resistor
  HAL_GPIO_Init(GPIOA, &GPIOPin); // initialize PA8 as analog input pin

  //Pin PA4 Set as digital input to monitor hall effect sensor output
/*   GPIO_InitTypeDef GPIOPin2;
  GPIOPin.Pin = GPIO_PIN_4; // Select pin PA4
  GPIOPin.Mode = GPIO_MODE_INPUT; // Select Digital input
  GPIOPin.Pull = GPIO_PULLUP; // Disable internal pull-up or pull-down resistor
  HAL_GPIO_Init(GPIOA, &GPIOPin2); // initialize PA8 as analog input pin */

  //Pin PB5/D11 Set as digital input to monitor hall effect sensor output
  GPIO_InitTypeDef GPIOPin2;
  GPIOPin.Pin = GPIO_PIN_5; // Select pin PB5/D11
  GPIOPin.Mode = GPIO_MODE_INPUT; // Select Digital input
  GPIOPin.Pull = GPIO_PULLUP; // Disable internal pull-up or pull-down resistor
  HAL_GPIO_Init(GPIOB, &GPIOPin2); // initialize PA8 as analog input pin

}

void LED_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
}


void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
 
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1) {}
}


void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void)
{
}


void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}