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

#define LED_PIN GPIO_PIN_3

#define LED_GPIO_PORT GPIOB

void USART2_Init(void);
void LED_Init(void);
void ADC_Init(void);
void SystemClock_Config(void);

static UART_HandleTypeDef h_UARTHandle;
static ADC_HandleTypeDef h_ACDC1Handle;

uint32_t ADC_Value = 1234;
char ADC_String[5];
HAL_StatusTypeDef err;

int main(void)
{   

  SystemClock_Config();
  HAL_Init();
  SystemCoreClockUpdate();

  USART2_Init();
  LED_Init();
  ADC_Init();

  while(1)
  {
    strcpy(ADC_String, "");
    HAL_ADC_Start(&h_ACDC1Handle);
    HAL_Delay(4);

    if(HAL_ADC_PollForConversion(&h_ACDC1Handle, 10) == HAL_OK) //Check that conversion completes
    {
      ADC_Value = HAL_ADC_GetValue(&h_ACDC1Handle); //Store ADC value
    }
    HAL_ADC_Stop(&h_ACDC1Handle); // stop conversion 
    
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
    

    HAL_UART_Transmit(&h_UARTHandle,(uint8_t*)ADC_String,strlen(ADC_String)-1,HAL_MAX_DELAY);
    
    HAL_Delay(100);
  }

  return 0;
}

void USART2_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_9; //TX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1; 
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
  
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1; //RX
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  h_UARTHandle.Instance        = USART1;
  h_UARTHandle.Init.BaudRate   = 9600;
  h_UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
  h_UARTHandle.Init.StopBits   = UART_STOPBITS_1;
  h_UARTHandle.Init.Parity     = UART_PARITY_NONE;
  h_UARTHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  h_UARTHandle.Init.Mode       = UART_MODE_TX_RX;

  HAL_UART_Init(&h_UARTHandle);
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
  Channel_AN.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // set sampling time to 15 clock cycles
  Channel_AN.OffsetNumber = ADC_OFFSET_NONE;
  Channel_AN.SingleDiff = ADC_SINGLE_ENDED; //Single ended mode possibly changes the ADC reference points to Ground and ADD?
  Channel_AN.Offset = 0;
  HAL_ADC_ConfigChannel(&h_ACDC1Handle, &Channel_AN); // select channel_0 for ADC2 module.
  
}


void LED_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(LED_GPIO_PORT,&GPIO_InitStruct);

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