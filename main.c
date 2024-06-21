/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define size_of_buffer 8
#define BITLENGTH 8 //1 Byte
//#define FIRSTBIT pow(2,(BITLENGTH-1))//MSB Value
#define FIRSTBIT (1 << (BITLENGTH - 1))
#define TRUE 1
#define CPU_FREQUENCY_MHZ 80
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
unsigned char UART1_rxBuffer[size_of_buffer] = {0};
unsigned char UART2_rxBuffer[size_of_buffer] = {0};
//uint8_t UART2_rxBuffer_2[size_of_buffer] = {0};
uint8_t txMessage[2]="1;";
uint8_t newlineN[8]="\r\n";
typedef struct {
    int result;
    int RXn;
} SC_Estimator_Result;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MLC(const uint8_t *pDa1, const uint8_t *pDa2);
int readByte(void);
int find_best_CT(unsigned char *a, unsigned char *b, unsigned char *c, int target);
void CT_engine(unsigned char *Da1, unsigned char *Da2, unsigned char *Da3, int check_packet);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void writeByte(char byte);
void BER_calc(const uint8_t *pDa);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
 // HAL_UART_Receive_DMA (&huart2, UART1_rxBuffer, size_of_buffer);
 // HAL_UART_Receive_DMA (&huart1, UART2_rxBuffer, size_of_buffer);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // HAL_UART_Receive_DMA (&huart2, UART_rxBuffer, size_of_buffer);
	 // HAL_UART_Transmit(&huart1, UART_rxBuffer, size_of_buffer, 100);
	//  HAL_UART_Transmit_DMA(&huart1, UART_rxBuffer, size_of_buffer);
	//  HAL_UART_Transmit(&huart1, txMessage, 2, 100);
	//  HAL_UART_Transmit(&huart2, txMessage, 2, 100);
	  HAL_UART_Receive_DMA (&huart2, UART2_rxBuffer, size_of_buffer);
	  //HAL_UART_Receive_DMA (&huart2, UART2_rxBuffer_2, size_of_buffer);
	  HAL_UART_Receive_DMA (&huart1, UART1_rxBuffer, size_of_buffer);
	 // MLC(UART1_rxBuffer,UART2_rxBuffer);
	// writeByte(255);
	  CT_engine(&UART2_rxBuffer[0], &UART2_rxBuffer[1], &UART1_rxBuffer[0], 255);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart2)
	{
    HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, size_of_buffer);
    HAL_UART_Transmit(&huart2, UART2_rxBuffer, size_of_buffer, 100);}
    //BER_calc(UART_rxBuffer);
    //HAL_UART_Transmit(&huart2, newlineN, size_of_buffer, 100);
	else if(huart==&huart1)
	{
	HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, size_of_buffer);
    HAL_UART_Transmit(&huart1, UART1_rxBuffer, size_of_buffer, 100);
    HAL_UART_Transmit(&huart2, UART1_rxBuffer, size_of_buffer, 100);}
}

//Sending 1byte number
void writeByte(char decimal)
{
//converting the decimal value to binary and sending 8 bit information from MSB to LSB
 int i,binary;
 for(i=0;i<BITLENGTH;i++)
 {

 binary = (int)decimal/FIRSTBIT; //Getting the first binary bit value
 decimal= (decimal & ((int)FIRSTBIT -1));//Setting the first bit to zero
 decimal=decimal<<1; //Shift all bits by one to left
 if(binary==TRUE)
 //if (a==1)
 {
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
  //analogWrite(A0,255);
 //Serial.print("1");
 }
 else
 {
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
 // analogWrite(A0,0);
 //Serial.print("0");
 }
 //delay(DELAY);
 HAL_Delay(1);
//Serial.println();
//digitalWrite(LED,LOW);
}
}

int readByte()
{
int i,input=0;
for(i=0;i<BITLENGTH;i++)
{//Repeat for each bit
// Read the state of the GPIO pin
GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14);
//int val = digitalRead(ledPin);
//Serial.println(sensorValue);
if(pinState == GPIO_PIN_SET)
{
input++;
}
//Binary shift to store another bit
input=input<<1;
//delay(DELAY);
//delayMicroseconds(1000);
HAL_Delay(1);
}
//Return the value if input
input=input>>1;
return input;
}


void delayMicroseconds(uint32_t us) {
    // Convert microseconds to CPU cycles
    uint32_t cycles = (us * CPU_FREQUENCY_MHZ) / 1000000;

    // Wait for the specified number of CPU cycles
    for (uint32_t i = 0; i < cycles; ++i) {
        __NOP(); // No operation (assembly instruction)
    }
}

void BER_calc(const uint8_t *pDa)
{
	 static int count1, count0,count;uint8_t BER;uint8_t BDR;
 if(*pDa==1)
 //if (a==1)
 {
	 count1++;
	 count++;
	 if(count>=100){BDR=(count1/100);HAL_UART_Transmit(&huart2, &BDR, 1, 100);count1=0;count=0;count0=0;BDR=0;BER=0;}
 }
 else
 {
	 count0++;count++;
	 if(count>=100){BER=(count0/100);HAL_UART_Transmit(&huart2, &BER, 1, 100);count1=0;count=0;count0=0;BDR=0;BER=0;}
 }

}

void MLC(const uint8_t *pDa1,const uint8_t *pDa2)
{
	uint8_t data = (*pDa1 + *pDa2) / 2;
	HAL_UART_Transmit(&huart2, &data, size_of_buffer, 100);
}

int MLC_Estimator()
{
	int count = 0;
	//int countRX_1 = 0;
	int count1=0,count0=0;
	   // int countRX_2 = 0;
	    int MLC_count1=0; int MLC_count0=0;
	  while (count < 8)
	  {
       HAL_UART_Receive_DMA (&huart2, UART1_rxBuffer, size_of_buffer);
       HAL_UART_Receive_DMA (&huart1, UART2_rxBuffer, size_of_buffer);
       if (UART1_rxBuffer[0] == 1)
       {
       //countRX_1++;
       count1++;
       }
       else {count0++;}
       if (UART2_rxBuffer[0] == 1)
          {
          //countRX_2++;
          count1++;
          }
       else{count0++;}
       if(count1>count0){MLC_count1++;}else if (count0>count1){MLC_count0++;}count1=0;count0=0;
       count++;
       }
	  return (MLC_count1/8);
}

SC_Estimator_Result SC_Estimator()
{
	int count = 0;
	int countRX_1 = 0;
	//int count1=0,
	int countRX_0=0;
	    int countRX_2 = 0;
	  //  int MLC_count1=0; int MLC_count0=0;
	  while (count < 8)
	  {
       HAL_UART_Receive_DMA (&huart2, UART1_rxBuffer, size_of_buffer);
       HAL_UART_Receive_DMA (&huart1, UART2_rxBuffer, size_of_buffer);
       if (UART1_rxBuffer[0] == 1)
       {
       countRX_1++;
       //count1++;
       }
       else {countRX_0++;}
       if (UART2_rxBuffer[0] == 1)
          {
          countRX_2++;
          //count1++;
          }
       else{countRX_0++;}

       count++;
       }
	  SC_Estimator_Result result;
	  if((countRX_1/8)>(countRX_2/8)){result.result = countRX_1 / 8;result.RXn=1;}
	  else if ((countRX_2/8)>(countRX_1/8))
	  { result.result = countRX_2 / 8;result.RXn = 2;}
	  else
	  {result.result = 0;
      result.RXn = 0;}
	  return result;
	  countRX_1=0;countRX_2=0;countRX_0=0;

}

int EGC_Estimator()
{
	int count = 0;
		    int EGC_count1=0; int EGC_count0=0;
		  while (count < 8)
		  {
	       HAL_UART_Receive_DMA (&huart2, UART1_rxBuffer, size_of_buffer);
	       HAL_UART_Receive_DMA (&huart1, UART2_rxBuffer, size_of_buffer);
	       // Assuming UART1_rxBuffer and UART2_rxBuffer are arrays of uint8_t and we need the first byte
	               int average = (UART1_rxBuffer[0] + UART2_rxBuffer[0]) / 2;

	               if (UART1_rxBuffer[0] >= average)
	               {
	                   EGC_count1++;
	               }
	               else if (UART1_rxBuffer[0] < average)
	               {
	                   EGC_count0++;
	               }

	               if (UART2_rxBuffer[0] >= average)
	               {
	                   EGC_count1++;
	               }
	               else if (UART2_rxBuffer[0] < average)
	               {
	                   EGC_count0++;
	               }

	               count++;average=0;
	           }

	           return EGC_count1 / 8;
}

int find_best_CT(unsigned char *a, unsigned char  *b, unsigned char  *c, int target) {
    // Calculate the mean of the three numbers
	unsigned char  mean_value = (*a + *b + *c) / 3;

    // Calculate the absolute difference between the mean and the target number
    int absolute_difference_mean = abs(mean_value - target); // token 0 (MLC)

    // Calculate the absolute differences from the target
    int differences[3][2] = {
        {*a, abs(*a - target)},
        {*b, abs(*b - target)},
        {*c, abs(*c - target)}
    };

    // Find the number with the smallest difference from the target
    int absolute_difference_least_variance = differences[0][1];
    for (int i = 1; i < 3; i++) {
        if (differences[i][1] < absolute_difference_least_variance) {
            absolute_difference_least_variance = differences[i][1];
        }
    }

    // Sort the differences based on the absolute difference
    for (int i = 0; i < 2; i++) {  // Loop until the second to last element
        for (int j = i + 1; j < 3; j++) {
            if (differences[i][1] > differences[j][1]) {
                // Swap
                int temp_num = differences[i][0];
                int temp_diff = differences[i][1];
                differences[i][0] = differences[j][0];
                differences[i][1] = differences[j][1];
                differences[j][0] = temp_num;
                differences[j][1] = temp_diff;
            }
        }
    }

    // Get the two numbers with the smallest differences from the target
    int least_variance_num1 = differences[0][0];
    int absolute_difference_least_variance1 = differences[0][1];
    int least_variance_num2 = differences[1][0];
    // int absolute_difference_least_variance2 = differences[1][1];
    int sum_least_two_num = (least_variance_num1 + least_variance_num2) / 2;

    // Calculate the absolute difference between the sum_least_two_num and the target number
    int absolute_difference_mean_sum_least_two_num = abs(sum_least_two_num - target);  // token 2 (EGC)

    // Compare the three absolute differences
    if (absolute_difference_mean < absolute_difference_least_variance1 && absolute_difference_mean < absolute_difference_mean_sum_least_two_num) {
        return 0; // MLC
    } else if (absolute_difference_least_variance1 < absolute_difference_mean && absolute_difference_least_variance1 < absolute_difference_mean_sum_least_two_num) {
        return 1; // SC
    } else {
        return 2; // EGC
    }
}


unsigned char MLC_Engine (unsigned char *a, unsigned char  *b, unsigned char  *c) {

	    // Calculate the mean of the three numbers
	    unsigned char  mean_value = (*a + *b + *c) / 3;
	    return mean_value;
}

int SC_Engine (unsigned char *a, unsigned char  *b, unsigned char  *c, int target) {

	    // Calculate the absolute differences from the target
	    int differences[3][2] = {{*a, abs(*a - target)}, {*b, abs(*b - target)}, {*c, abs(*c - target)}};

	    // Find the number with the smallest difference from the target
	    int least_variance_num = differences[0][0];
	    int absolute_difference_least_variance = differences[0][1];
	    for (int i = 1; i < 3; i++) {
	        if (differences[i][1] < absolute_difference_least_variance) {
	            least_variance_num = differences[i][0];
	        }
	    }
	    return least_variance_num;
}

int EGC_Engine (unsigned char *a, unsigned char  *b, unsigned char  *c, int target) {

	    // Calculate the absolute differences from the target
	    int differences[3][2] = {{*a, abs(*a - target)}, {*b, abs(*b - target)}, {*c, abs(*c - target)}};

	    // Sort the differences based on the absolute difference
	    for (int i = 0; i < 3; i++) {
	        for (int j = i + 1; j < 3; j++) {
	            if (differences[i][1] > differences[j][1]) {
	                // Swap
	                int temp[2];
	                temp[0] = differences[i][0];
	                temp[1] = differences[i][1];
	                differences[i][0] = differences[j][0];
	                differences[i][1] = differences[j][1];
	                differences[j][0] = temp[0];
	                differences[j][1] = temp[1];
	            }
	        }
	    }

	    // Get the two numbers with the smallest differences from the target
	    int least_variance_num1 = differences[0][0];
	    int least_variance_num2 = differences[1][0];
	    int sum_least_two_num = (least_variance_num1 + least_variance_num2) / 2;

	    // Calculate the absolute difference between the sum_least_two_num and the target number
	    int absolute_difference_mean_sum_least_two_num = abs(sum_least_two_num - target);  // token 2 (EGC)
	    return absolute_difference_mean_sum_least_two_num;
}

void CT_engine(unsigned char *Da1, unsigned char *Da2, unsigned char *Da3, int check_packet){
    int result = find_best_CT(Da1, Da2, Da3, check_packet);
    unsigned char Rec_data = 0;

    HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, size_of_buffer);
    HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, size_of_buffer);

    switch(result) {
        case 0:
            Rec_data = MLC_Engine(&UART2_rxBuffer[0], &UART2_rxBuffer[1], &UART1_rxBuffer[0]);
            break;
        case 1:
            Rec_data = SC_Engine(&UART2_rxBuffer[0], &UART2_rxBuffer[1], &UART1_rxBuffer[0], 255);
            break;
        case 2:
            Rec_data = EGC_Engine(&UART2_rxBuffer[0], &UART2_rxBuffer[1], &UART1_rxBuffer[0], 255);
            break;
    }

    HAL_UART_Transmit(&huart2, &Rec_data, 1, 100);
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
