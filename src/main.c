#include <stdio.h>
#include <stm32l4xx_hal.h>
#include <string.h>

#define LED_PIN GPIO_PIN_3

#define LED_GPIO_PORT GPIOB

void USART2_Init(void);
void LED_Init(void);
void ADC_Init(void);

static UART_HandleTypeDef h_UARTHandle;
static ADC_HandleTypeDef h_ACDC1Handle;

uint32_t ADC_Value = 1234;
char ADC_String[5];
HAL_StatusTypeDef err;

int main(void)
{   

  HAL_Init();
  SystemCoreClockUpdate();

  USART2_Init();
  LED_Init();
  ADC_Init();

  while(1)
  {
    strcpy(ADC_String, "");
    HAL_ADC_Start(&h_ACDC1Handle);
    HAL_Delay(2);

    if(HAL_ADC_PollForConversion(&h_ACDC1Handle, 5) == HAL_OK) //Check that conversion completes
    {
      ADC_Value = HAL_ADC_GetValue(&h_ACDC1Handle); //Store ADC value
    }
    HAL_ADC_Stop(&h_ACDC1Handle); // stop conversion 
    
    itoa(ADC_Value, ADC_String, 10);

    HAL_UART_Transmit(&h_UARTHandle,(uint8_t*)"1000",strlen("1000"),HAL_MAX_DELAY);
    //HAL_UART_Transmit(&h_UARTHandle,(uint8_t*)ADC_String,strlen(ADC_String),HAL_MAX_DELAY);
    
    HAL_Delay(10);
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
  ADCpin.Pin = GPIO_PIN_3; // Select pin PA3
  ADCpin.Mode = GPIO_MODE_ANALOG; // Select Analog Mode
  ADCpin.Pull = GPIO_NOPULL; // Disable internal pull-up or pull-down resistor
  HAL_GPIO_Init(GPIOA, &ADCpin); // initialize PA0 as analog input pin

  //INIT ADC 
  h_ACDC1Handle.Instance = ADC1; // create an instance of ADC1
  h_ACDC1Handle.Init.Resolution = ADC_RESOLUTION_12B; // select 12-bit resolution
  h_ACDC1Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //select  single conversion as a end of conversion event
  h_ACDC1Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT; // set digital output data right justified
  h_ACDC1Handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  
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
  ADC_ChannelConfTypeDef Channel_AN5; // create an instance of ADC_ChannelConfTypeDef
  Channel_AN5.Channel = ADC_CHANNEL_8; // select analog channel 8 (ADC1_IN8)
  Channel_AN5.Rank = 1; // set rank to 1
  Channel_AN5.SamplingTime = ADC_SAMPLETIME_2CYCLES_5; // set sampling time to 15 clock cycles
  Channel_AN5.OffsetNumber = ADC_OFFSET_NONE;
  Channel_AN5.Offset = 0;
  HAL_ADC_ConfigChannel(&h_ACDC1Handle, &Channel_AN5); // select channel_0 for ADC2 module.
  
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