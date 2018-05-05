
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "KS0108.h"
#include "glcd_menu.h"
#include "Dsp.h"
#include "PID.h"
#include "defines.h"
//#include <math.h>
#include <stdlib.h>
//#include <string.h>
//#include "font5x8.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

TaskHandle_t lcd_demo_handle = NULL;
TaskHandle_t read_adc_handle = NULL;
Menu menu_list[15];
uint8_t uart_buff[20];
int active_menu=0;
uint32_t adc_value[2*bufferLength]={0};
uint16_t pH_buffer[bufferLength];
uint16_t temp_buffer[bufferLength];
// moving variables
uint16_t ph_history[filterWindowLength]={0};
int32_t ph_sum=0;
uint16_t ph_window_pointer=0;
uint16_t temp_history[filterWindowLength]={0};
int32_t temp_sum=0;
uint16_t temp_window_pointer=0;
float p1_p =  0.004076;
float p2_p = -0.8505;
float p1_t = 0.04082;
float p2_t = 2.204;
float *pH = &menu_list[0].values[0];
float *temp = &menu_list[0].values[2];
uint8_t message[30];
uint8_t log_uart[10];
uint16_t tempDigital = 0;
uint16_t ph_averaged = 0;
float *pH_filtered = &menu_list[2].values[3];
uint16_t temp_filtered = 0;
float *calibration_point_1_ph = &menu_list[2].values[1];
float *calibration_point_2_ph = &menu_list[3].values[1];
float *calibration_point_1 = &menu_list[2].values[3];
float *calibration_point_2 = &menu_list[3].values[2];


// rtc 
//RTC_TimeTypeDef rtc_time;
float *rtc_minute_p = &menu_list[8].values[0];
float *rtc_hour_p = &menu_list[8].values[1];
float *rtc_day_p = &menu_list[8].values[2];
float *rtc_month_p = &menu_list[8].values[3];
float *rtc_year_p = &menu_list[8].values[4];

// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata;
pid_t pid;
// Control loop input,output and setpoint variables
float input = 0;
float *output = &menu_list[0].values[3];
float *setpoint = &menu_list[5].values[2];
// Control loop gains
float kp = 100, ki =1.2, kd = 0;
// PID sample time in ms
uint32_t sample_time = 10;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void lcd_demo(void *);
void read_adc(void *);
void pump_set_stroke(uint16_t stroke);
//extern void calculate_calibration_coefficients(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	// Prepare PID controller for operation
	pid = pid_create(&ctrldata, &input, output, setpoint, kp, ki, kd);
	// Set controler output limits from 0 to 90
	pid_limits(pid, 0, 90);
	// Allow PID to compute and change output
	pid_auto(pid);
	// Set sampling time
	pid_sample(pid, sample_time);
	// Set the direction of controller
	pid_direction(pid, E_PID_REVERSE);
	
	GLCD_Initalize();
	GLCD_ClearScreen();
	HAL_UART_Transmit(&huart2,"init2\n",6,100);
	init_menu();
	HAL_UART_Transmit(&huart2,"init3\n",6,100);
	HAL_UART_Receive_IT(&huart2,uart_buff,1);
	HAL_UART_Transmit(&huart2,"init4\n",6,100);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Transmit(&huart2,"init5\n",6,100);
	HAL_ADC_Start_DMA(&hadc1, adc_value, 2*bufferLength);
	
	xTaskCreate(lcd_demo,"lcd_demo",128,( void * )1,1,&lcd_demo_handle);
	xTaskCreate(read_adc,"read_adc",256,( void * )1,5,&read_adc_handle);


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */
void lcd_demo(void * pvParameters)
{
	while(1)
	{		
		if(active_menu == 0)
		{
			print_main_page(active_menu);
			if(active_menu != 0)
			{
				print_menu(active_menu);
			}
		}
		osDelay(1000);
		//HAL_Delay(1000);
	}
}
void read_adc(void * pvParameters)
{
	while(1)
	{		
		
		osDelay(1000);
		//HAL_Delay(1000);
	}
}
/**
  * @brief  This function sets pump stroke.
	* @param  stroke: stroke value from 0 to 100.
  * @retval None
  */ 
void pump_set_stroke(uint16_t stroke)
{
	uint16_t dac_value = (4095 * ((4 + stroke * 0.16) * 0.1) / 3.3);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);
}

void calculate_calibration_coefficients(void)
{
	p1_p = abs((int)(*calibration_point_1_ph - *calibration_point_2_ph)) / abs((int)(*calibration_point_1 - *calibration_point_2 )) ;
	p2_p = *calibration_point_1_ph - (*calibration_point_1)*(p1_p);
	//sprintf(sprintf_buff,"value:%f\n",menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer]);
			HAL_UART_Transmit(&huart2,"calibration\n",10,100);
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Stop_DMA(&hadc1);
	data_spliter(adc_value, bufferLength, temp_buffer, pH_buffer);
	ph_averaged = average(pH_buffer, bufferLength);
	*pH_filtered = best_moving_average(ph_averaged, ph_history, &ph_sum, &ph_window_pointer);
	*pH = p1_p * *pH_filtered + p2_p;
	tempDigital = average(temp_buffer, bufferLength);
	temp_filtered = best_moving_average(tempDigital, temp_history, &temp_sum, &temp_window_pointer);
	*temp = p1_t * temp_filtered + p2_t;
	/* Updating PID input*/
	input = *pH;
	/* Compute new PID output*/
	pid_compute(pid);
	char sprintf_buff[12];
	//sprintf(sprintf_buff,"%.3f,%.1f\n",*pH,*output);
	//HAL_UART_Transmit(&huart2,sprintf_buff,12,100);
//	for(int i=0;i<bufferLength;i++)
//	{
//		sprintf(sprintf_buff,"%d,",pH_buffer[i]);
//		HAL_UART_Transmit(&huart2,sprintf_buff,6,100);
//	}
//	HAL_UART_Transmit(&huart2,"\n",1,100);
//	for(int i=0;i<bufferLength;i++)
//	{
//		sprintf(sprintf_buff,"%d,",temp_buffer[i]);
//		HAL_UART_Transmit(&huart2,sprintf_buff,6,100);
//	}
//	HAL_UART_Transmit(&huart2,"\n",1,100);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Transmit(&huart2,"ok\n",3,100);
	
	get_user_input(uart_buff,&active_menu);
	if(active_menu != 0)
	{
	update_menu_from_variables();
	print_menu(active_menu);
	}
	HAL_UART_Transmit(&huart2,uart_buff,1,100);
	HAL_UART_Receive_IT(&huart2,uart_buff,1);
}
/* USER CODE END 4 */

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
	if (htim->Instance == TIM2) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value,2*bufferLength);
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
