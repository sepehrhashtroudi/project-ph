
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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "KS0108.h"
#include "glcd_menu.h"
#include "Dsp.h"
#include "PID.h"
#include "defines.h"
#include "max485.h"
#include "eeprom.h"
#include <stdlib.h>
#include "math.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

TaskHandle_t lcd_demo_handle = NULL;
TaskHandle_t main_thread_handle = NULL;
TaskHandle_t ph_calibration_thread_handle = NULL;
TaskHandle_t temp_calibration_thread_handle = NULL;
SemaphoreHandle_t lcd_semaphore;
int create_ph_calibration_task_flag = 0;
int delete_ph_calibration_task_flag = 0;
int create_temp_calibration_task_flag = 0;
int delete_temp_calibration_task_flag = 0;
Menu menu_list[20];
Menu last_menu_list[20];
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
float pH =0;
float temp;
int32_t eeprom_buff;

uint8_t message[30];
uint8_t log_uart[10];
uint16_t ph_averaged = 0;
uint16_t pH_filtered ;
uint16_t temp_averaged = 0;
uint16_t temp_filtered = 0;
int16_t progress = 0;
int16_t last_progress = 0;
int16_t pump_on_off_state = 0;
// rtc 
RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;
float *rtc_hour_p = &menu_list[0].values[3];
float *rtc_minute_p = &menu_list[0].values[4];
// Structure to store PID data and pointer to PID structure
struct pid_controller ctrldata;
pid_t pid;
// Control loop input,output and setpoint variables
float input = 0;
float output ;
float *setpoint = &menu_list[8].values[3];
// Control loop gains
float *kp = &menu_list[10].values[0];
float *ki = &menu_list[10].values[1];
float *kd = &menu_list[10].values[2];
// PID sample time in ms
uint32_t sample_time = 10;
extern const unsigned char Times_New_Roman25x26[] ;
static FLASH_EraseInitTypeDef EraseInitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void main_thread(void *);
void lcd_print(void *);
void read_adc(void *);
void ph_calibration_thread(void *);
void temp_calibration_thread(void * pvParameters);
void pump_set_stroke(uint16_t stroke);
void read_setting_from_eeprom(void);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	init_menu();
	read_setting_from_eeprom();
//	eeprom_buff = p1_p*float_to_int_factor;
//	eeprom_write_data(p1_p_eeprom_add,&eeprom_buff,1);
//	eeprom_buff = p2_p*float_to_int_factor;
//	eeprom_write_data(p2_p_eeprom_add,&eeprom_buff,1);
	// Prepare PID controller for operation
	pid = pid_create(&ctrldata, &input, &output, setpoint, *kp, *ki, *kd);
	// Set controler output limits from 0 to 100
	pid_limits(pid, 0, 100);
	// Allow PID to compute and change output
	pid_auto(pid);
	// Set sampling time
	pid_sample(pid, sample_time);
	// Set the direction of controller
	pid_direction(pid, E_PID_REVERSE);
	
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	char sprintf_buff[20];
	MAX485_send_string("starting ...",14,100);
	HAL_UART_Receive_IT(&huart3,uart_buff,1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(&hadc1, adc_value, 2*bufferLength);
	GLCD_Initalize();
	GLCD_ClearScreen();
	glcd_set_font_with_num(1);
	glcd_draw_string_xy(10,10,"Loading...",0,0,0);
	lcd_semaphore = xSemaphoreCreateCounting(5,1);
	xTaskCreate(lcd_print,"lcd_print",64,( void * )1,3,&lcd_demo_handle);
	xTaskCreate(main_thread,"main_thread",512,( void * )1,2,&main_thread_handle);
	//xTaskCreate(calibration_thread,"calibration_thread",128,( void * )1,3,&calibration_thread_handle);
	


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
void main_thread(void * pvParameters)
{	
	uint32_t last_lcd_init=0;
	HAL_Delay(100);
	GLCD_Initalize();
	GLCD_ClearScreen();
	while(1)
	{
		HAL_RTC_GetTime(&hrtc,&rtc_time,FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc,&rtc_date,FORMAT_BIN);
		*rtc_minute_p = rtc_time.Minutes;
		*rtc_hour_p = rtc_time.Hours;
		
		if( xSemaphoreTake( lcd_semaphore, 1000 ) == pdTRUE )
		{
			if((uint32_t)(HAL_GetTick() - last_lcd_init) > 3600000)
			{
				GLCD_Initalize();
				last_lcd_init = HAL_GetTick();
			}
			update_menu_from_variables();
			if(active_menu == 0 )
			{
				print_main_page(active_menu);
			}
			else 
			{
				print_menu(active_menu);
			}
			
		}
		
		if(create_ph_calibration_task_flag == 1)
		{
			create_ph_calibration_task_flag = 0;
			progress = 0;
			last_progress = 0;
			xTaskCreate(ph_calibration_thread,"ph_calibration_thread",128,( void * )1,3,&ph_calibration_thread_handle);
		}
		if(delete_ph_calibration_task_flag == 1)
		{
			delete_ph_calibration_task_flag = 0;
			if( ph_calibration_thread_handle != NULL )
			{
				vTaskDelete( ph_calibration_thread_handle );
				ph_calibration_thread_handle = NULL;
				progress = 0;
				last_progress = 0;
			}
		}
		if(create_temp_calibration_task_flag == 1)
		{
			create_temp_calibration_task_flag = 0;
			progress = 0;
			last_progress = 0;
			xTaskCreate(temp_calibration_thread,"temp_calibration_thread",128,( void * )1,3,&temp_calibration_thread_handle);
		}
		if(delete_temp_calibration_task_flag == 1)
		{
			delete_temp_calibration_task_flag = 0;
			if( temp_calibration_thread_handle != NULL)
			{
				vTaskDelete( temp_calibration_thread_handle );
				temp_calibration_thread_handle = NULL;
				progress = 0;
				last_progress = 0;
			}
		}
	}
}

void lcd_print(void * pvParameters)
{
	while(1)
	{
		osDelay(2000);
		update_menu_from_variables();
		if(active_menu == 0 )
		{
			if( menu_list[0].values[0] != last_menu_list[0].values[0] || menu_list[0].values[2] != last_menu_list[0].values[2] || menu_list[0].values[3] != last_menu_list[0].values[3] || menu_list[0].values[4] != last_menu_list[0].values[4] || menu_list[0].values[5] != last_menu_list[0].values[5])
			{
				xSemaphoreGive( lcd_semaphore );
				last_menu_list[0] = menu_list[0];
			}
		}
		if( active_menu == 3 || active_menu == 5 || active_menu == 13 || active_menu == 15 )
		{ 
				if(progress - last_progress >= 25 || progress == 100)
				{
					xSemaphoreGive( lcd_semaphore );
					last_progress = progress;
				}
		}
	}
}
void ph_calibration_thread(void * pvParameters)
{
	uint16_t last_ph_filtered = pH_filtered;
	uint32_t start_time = HAL_GetTick();
	char sprintf_buff[20];
	while(1)
	{
		MAX485_send_string("ph calibration\n",15,100);
		progress = (HAL_GetTick() - start_time)/1000 - (abs(pH_filtered - last_ph_filtered)) ;
		sprintf(sprintf_buff,"%d\n",pH_filtered);
		MAX485_send_string(sprintf_buff,7,100);
		if(abs(pH_filtered - last_ph_filtered) > 3)
		{
			start_time = HAL_GetTick();
			MAX485_send_string("reset\n",6,100);
		}
		if(progress > 100 )
		{
			progress = 100;
		}
		if(progress < 0 )
		{
			progress = 0;
		}
		last_ph_filtered = pH_filtered;
		osDelay(3000);
	}
}

void temp_calibration_thread(void * pvParameters)
{
	char sprintf_buff[10];
	uint16_t last_temp_filtered = temp_filtered;
	uint32_t start_time = HAL_GetTick();
	while(1)
	{
		MAX485_send_string("temp calibration\n",20,100);
		sprintf(sprintf_buff,"%d\n",temp_filtered);
		MAX485_send_string(sprintf_buff,7,100);
		progress = (HAL_GetTick() - start_time)/1000 - (abs(temp_filtered - last_temp_filtered)) ;
		if(abs(temp_filtered - last_temp_filtered) > 3)
		{
			start_time = HAL_GetTick();
			MAX485_send_string("reset\n",10,100);
		}
		if(progress > 100 )
		{
			progress = 100;
		}
		if(progress < 0 )
		{
			progress = 0;
		}
		last_temp_filtered = temp_filtered;
		osDelay(3000);
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
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
}
void send_pH_4_20(uint16_t pH)
{
	uint16_t dac_value = (4095 * ((4 + pH * 16.0 / 14.0) * 0.1) / 3.3);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);
}
void pump_turn_on_off(uint16_t state)
{
	if(state == 1)
	{
		HAL_GPIO_WritePin(REL_1_GPIO_Port,REL_1_Pin,GPIO_PIN_SET);
	}
	if(state == 0)
	{
		HAL_GPIO_WritePin(REL_1_GPIO_Port,REL_1_Pin,GPIO_PIN_RESET);
	}
}
void read_setting_from_eeprom(void)
{
	eeprom_read_data(p1_p_eeprom_add,&eeprom_buff,1);
	p1_p = eeprom_buff / float_to_int_factor;
	eeprom_read_data(p2_p_eeprom_add,&eeprom_buff,1);
	p2_p = eeprom_buff / float_to_int_factor;
	eeprom_read_data(p1_t_eeprom_add,&eeprom_buff,1);
	p1_t = eeprom_buff / float_to_int_factor;
	eeprom_read_data(p2_t_eeprom_add,&eeprom_buff,1);
	p2_t = eeprom_buff / float_to_int_factor;
	eeprom_read_data(pid_p_eeprom_add,&eeprom_buff,1);
	menu_list[10].values[0] = eeprom_buff / float_to_int_factor;
	eeprom_read_data(pid_i_eeprom_add,&eeprom_buff,1);
	menu_list[10].values[1] = eeprom_buff / float_to_int_factor;
	eeprom_read_data(pid_d_eeprom_add,&eeprom_buff,1);
	menu_list[10].values[2] = eeprom_buff / float_to_int_factor;
	eeprom_read_data(relay_max_eeprom_add,&eeprom_buff,1);
	menu_list[9].values[0] = eeprom_buff / float_to_int_factor;
	eeprom_read_data(relay_min_eeprom_add,&eeprom_buff,1);
	menu_list[9].values[1] = eeprom_buff / float_to_int_factor;
	eeprom_read_data(controller_on_off_eeprom_add,&eeprom_buff,1);
	menu_list[8].values[0] = eeprom_buff ;
	eeprom_read_data(controller_type_eeprom_add,&eeprom_buff,1);
	menu_list[8].values[1] = eeprom_buff ;
	eeprom_read_data(controller_setpoint_eeprom_add,&eeprom_buff,1);
	menu_list[8].values[3] = eeprom_buff / float_to_int_factor;


}
void set_date_time(void)
{
	rtc_time.Hours = menu_list[11].values[0];
	rtc_time.Minutes = menu_list[11].values[1];
	rtc_time.Seconds=1;
	HAL_RTC_SetTime(&hrtc,&rtc_time,FORMAT_BIN );
}
void set_pid_coefficients(void)
{
	pid = pid_create(&ctrldata, &input, &output, setpoint, *kp, *ki, *kd);
	// Set controler output limits from 0 to 90
	pid_limits(pid, 0, 100);
	// Allow PID to compute and change output
	pid_auto(pid);
	// Set sampling time
	pid_sample(pid, sample_time);
	// Set the direction of controller
	pid_direction(pid, E_PID_REVERSE);
	eeprom_buff = *kp * float_to_int_factor;
	eeprom_write_data(pid_p_eeprom_add,&eeprom_buff,1);
	eeprom_buff = *ki * float_to_int_factor;
	eeprom_write_data(pid_i_eeprom_add,&eeprom_buff,1);
	eeprom_buff = *kd * float_to_int_factor;
	eeprom_write_data(pid_d_eeprom_add,&eeprom_buff,1);
	eeprom_buff = menu_list[8].values[0];
	eeprom_write_data(controller_on_off_eeprom_add,&eeprom_buff,1);
	eeprom_buff = menu_list[8].values[1];
	eeprom_write_data(controller_type_eeprom_add,&eeprom_buff,1);
	eeprom_buff = menu_list[8].values[3]* float_to_int_factor;
	eeprom_write_data(controller_setpoint_eeprom_add,&eeprom_buff,1);

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Stop_DMA(&hadc1);
	data_spliter(adc_value, bufferLength,pH_buffer,temp_buffer );
	ph_averaged = average(pH_buffer, bufferLength);
	pH_filtered = best_moving_average(ph_averaged, ph_history, &ph_sum, &ph_window_pointer);
	pH = p1_p * pH_filtered + p2_p;
	pH = roundf(pH * 100.0) / 100.0; 
	if(pH > 14)
	{
		pH = 14;
	}
	if(pH < 0)
	{
		pH = 0;
	}
	send_pH_4_20(pH);
	temp_averaged = average(temp_buffer, bufferLength);
	temp_filtered = best_moving_average(temp_averaged, temp_history, &temp_sum, &temp_window_pointer);
	temp = p1_t * temp_filtered + p2_t;
	temp = roundf(temp * 10.0) / 10.0; 
	if(temp_filtered > 3180 || temp_filtered < 10)
	{
		strcpy(menu_list[0].menu_strings[2] , "NC `c");
	}
	else
	{
		strcpy(menu_list[0].menu_strings[2] , "%.1f `c");
	}
	
	if(menu_list[8].values[0]==0) // controller is ON
	{
		if(menu_list[8].values[1]==0) // controller type is pid
		{
			pump_turn_on_off(0);
			/* Updating PID input*/
			input = pH;
			/* Compute new PID output*/
			pid_compute(pid);
			pump_set_stroke(output);
		}
		else if(menu_list[8].values[1]==1)// controller type is relay
		{
			uint16_t threshold = (menu_list[9].values[0] - menu_list[9].values[1])/4;
			uint16_t mean = (menu_list[9].values[0] + menu_list[9].values[1])/2;
			if (pH > mean + threshold ) // pH is bigger than maximum value
			{
				pump_on_off_state = 1;
				pump_turn_on_off(pump_on_off_state);
			}
			else if (pH < mean - threshold )
			{
				pump_on_off_state = 0;
				pump_turn_on_off(pump_on_off_state);
			}
		}
	}
	else
	{
		pump_turn_on_off(0);
		output=0;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	get_user_input(uart_buff,&active_menu);
	xSemaphoreGiveFromISR( lcd_semaphore, NULL );
	//MAX485_send_string(uart_buff,13,100);
	HAL_UART_Receive_IT(&huart3,uart_buff,1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//HAL_GPIO_WritePin(POWER_STATE_GPIO_Port ,POWER_STATE_Pin , GPIO_PIN_RESET);
	for(int i=0; i<200000;i++);
	//HAL_GPIO_WritePin(POWER_STATE_GPIO_Port ,POWER_STATE_Pin , GPIO_PIN_SET);
	
	if(GPIO_Pin==Esc_Pin)
	{
		if(HAL_GPIO_ReadPin(Esc_GPIO_Port,Esc_Pin)==0)
		{
		//HAL_UART_Transmit(&huart2,"ESC\n",4,100);
			uart_buff[0]='e';
		}
	}
	
	if(GPIO_Pin==Enter_Pin)
	{
		if(HAL_GPIO_ReadPin(Enter_GPIO_Port,Enter_Pin)==0)
		{
		//HAL_UART_Transmit(&huart2,"ESC\n",4,100);
			uart_buff[0]='\n';
			
		}
	}
	if(GPIO_Pin==Up_Pin)
	{
		if(HAL_GPIO_ReadPin(Up_GPIO_Port,Up_Pin)==0)
		{
		//HAL_UART_Transmit(&huart2,"ESC\n",4,100);
			uart_buff[0]='w';
		}
	}
	if(GPIO_Pin==Down_Pin)
	{
		if(HAL_GPIO_ReadPin(Down_GPIO_Port,Down_Pin)==0)
		{
		//HAL_UART_Transmit(&huart2,"ESC\n",4,100);
			uart_buff[0]='s';
		}
	}
	if(GPIO_Pin==Right_Pin)
	{
		if(HAL_GPIO_ReadPin(Right_GPIO_Port,Right_Pin)==0)
		{
		//HAL_UART_Transmit(&huart2,"ESC\n",4,100);
			uart_buff[0]='d';
		}
	}
	if(GPIO_Pin==Left_Pin)
	{
		if(HAL_GPIO_ReadPin(Left_GPIO_Port,Left_Pin)==0)
		{
		//HAL_UART_Transmit(&huart2,"ESC\n",4,100);
			uart_buff[0]='a';
		}
	}
	get_user_input(uart_buff,&active_menu);
	xSemaphoreGiveFromISR( lcd_semaphore, NULL );
	//MAX485_send_string(uart_buff,13,100);
	
	
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
