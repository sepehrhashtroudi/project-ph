/**
  ******************************************************************************
  * @file    glcd_menu.c 
  * @brief   general menu for glcd.
  ******************************************************************************
	by sepehr hashtroudi
	sepehrhashtroudi@gmail.com
  ******************************************************************************
  */
#include "stm32f4xx_hal.h"
#include "max485.h"
#include "glcd_menu.h"
#include "eeprom.h"
#include "defines.h"
#include "tim.h"
#include <stdlib.h>
#include "cmsis_os.h"
extern SemaphoreHandle_t lcd_semaphore;
extern Menu menu_list[menu_list_length];
extern float p1_p;
extern float p2_p;
extern float p1_t;
extern float p2_t;
float slope ;
float zero ;
float slope_percent;
int32_t EEprom_buff;
float *calibration_point_1_ph = &menu_list[2].values[0];
float *calibration_point_2_ph = &menu_list[4].values[0];
float *calibration_point_1_temp = &menu_list[12].values[0];
float *calibration_point_2_temp = &menu_list[14].values[0];
int16_t calibration_point_1 = 0;
int16_t calibration_point_2 = 0;
float ph_calibration_start_temp = 0;
float ph_calibration_end_temp = 0;
extern float ph_calibration_temp ;
extern int create_ph_calibration_task_flag;
extern int delete_ph_calibration_task_flag;
extern int create_temp_calibration_task_flag;
extern int delete_temp_calibration_task_flag;
extern uint16_t pH_filtered;
extern uint16_t temp_filtered;
extern float temp;
extern int auto_wash_state;
extern int active_menu;
void ph_calculate_calibration_coefficients(void)
{

	ph_calibration_temp = (ph_calibration_start_temp + ph_calibration_end_temp)/2.0;
	EEprom_buff = ph_calibration_temp*float_to_int_factor;
	eeprom_write_data(ph_callibration_temp_add,&EEprom_buff,1);
	EEprom_buff = p1_p*float_to_int_factor;
	eeprom_write_data(p1_p_eeprom_add,&EEprom_buff,1);
	EEprom_buff = p2_p*float_to_int_factor;
	eeprom_write_data(p2_p_eeprom_add,&EEprom_buff,1);
}

void ph_calibration_step1(void)
{
	MAX485_send_string("calibration1\n",13,100);
	create_ph_calibration_task_flag = 1;
}
void ph_calibration_step2(void)
{
	MAX485_send_string("calibration2\n",13,100);
	create_ph_calibration_task_flag = 1;
}
void ph_calibration_waiting_1(void)
{
	delete_ph_calibration_task_flag = 1;
	calibration_point_1 = pH_filtered;
	ph_calibration_start_temp = temp;
}
void ph_calibration_waiting_2(void)
{
	
	delete_ph_calibration_task_flag = 1;
	calibration_point_2 = pH_filtered;
	ph_calibration_end_temp = temp;
	
	p1_p = (*calibration_point_1_ph - *calibration_point_2_ph) / (float)(calibration_point_1 - calibration_point_2 ) ;
	p2_p = *calibration_point_1_ph - (calibration_point_1)*(p1_p);
	slope = 1000 / (p1_p * 4095);
	zero = 1000 * (p2_p + 1.454)/ (p1_p * 4095);
	slope_percent = 100 * slope / 59.16;
	Change_Menu_Items(6,1,NULL,-1,slope,-1);
	Change_Menu_Items(6,2,NULL,-1,zero,-1);
	char sprintf_buff[20];
	sprintf(sprintf_buff,"%.5f,%.5f\n",slope,zero);
	MAX485_send_string(sprintf_buff,13,100);
	if( slope_percent < 90 || slope_percent > 105 || zero > 50 || zero < -50 )
	{
		MAX485_send_string("slope error",13,100);
	}
	
	
}

void temp_calculate_calibration_coefficients(void)
{
	char sprintf_buff[20];
	p1_t = (*calibration_point_1_temp - *calibration_point_2_temp) / (float)(calibration_point_1 - calibration_point_2 ) ;
	p2_t = *calibration_point_1_temp - (calibration_point_1)*(p1_t);
	EEprom_buff = p1_t*float_to_int_factor;
	eeprom_write_data(p1_t_eeprom_add,&EEprom_buff,1);
	EEprom_buff = p2_t*float_to_int_factor;
	eeprom_write_data(p2_t_eeprom_add,&EEprom_buff,1);
	sprintf(sprintf_buff,"%.5f,%.5f\n",p1_t,p2_t);
	MAX485_send_string(sprintf_buff,13,100);
}

void temp_calibration_step1(void)
{
	MAX485_send_string("calibration1\n",13,100);
	create_temp_calibration_task_flag = 1;
}
void temp_calibration_step2(void)
{
	MAX485_send_string("calibration2\n",13,100);
	create_temp_calibration_task_flag = 1;
}
void temp_calibration_waiting_1(void)
{
	delete_temp_calibration_task_flag = 1;
	calibration_point_1 = temp_filtered;
}
void temp_calibration_waiting_2(void)
{
	delete_temp_calibration_task_flag = 1;
	calibration_point_2 = temp_filtered;
}
void set_relay_Hysteresis(void)
{
	EEprom_buff = menu_list[9].values[0]*float_to_int_factor;
	eeprom_write_data(relay_max_eeprom_add,&EEprom_buff,1);
	EEprom_buff = menu_list[9].values[1]*float_to_int_factor;
	eeprom_write_data(relay_min_eeprom_add,&EEprom_buff,1);
	EEprom_buff = menu_list[8].values[0];
	eeprom_write_data(controller_on_off_eeprom_add,&EEprom_buff,1);
	EEprom_buff = menu_list[8].values[1];
	eeprom_write_data(controller_type_eeprom_add,&EEprom_buff,1);
}
void set_controller_set_point(void)
{
	EEprom_buff = menu_list[8].values[0];
	eeprom_write_data(controller_on_off_eeprom_add,&EEprom_buff,1);
	EEprom_buff = menu_list[8].values[1];
	eeprom_write_data(controller_type_eeprom_add,&EEprom_buff,1);
	EEprom_buff = menu_list[8].values[3]* float_to_int_factor;
	eeprom_write_data(controller_setpoint_eeprom_add,&EEprom_buff,1);
}

void relay_on_off(int func_num , int state)
{
	if(relay1_func == func_num)
	{
		HAL_GPIO_WritePin(REL_1_GPIO_Port, REL_1_Pin, state);
	}
	if(relay2_func == func_num)
	{
		HAL_GPIO_WritePin(REL_2_GPIO_Port, REL_2_Pin, state);
	}
	if(relay3_func == func_num)
	{
		//HAL_GPIO_WritePin(REL_3_GPIO_Port,REL_3_Pin,state);
	}
	if(relay4_func == func_num)
	{
		//HAL_GPIO_WritePin(REL_4_GPIO_Port,REL_4_Pin,state);
	}
}

void auto_wash_handler(int *auto_wash_state)
{

		if(*auto_wash_state == 0)
		{
			relay_on_off(supply_func_num , 1);  
			relay_on_off(drain_func_num , 0);
			relay_on_off(kcl_func_num , 0);
			relay_on_off(wash_func_num , 0);	
			Change_Menu_Items(AUTO_WASH_STATE_MENU,0,NULL,-1,1,0);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,1,NULL,-1,0,0);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,2,NULL,-1,0,0);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,3,NULL,-1,0,0); 
			xSemaphoreGiveFromISR ( lcd_semaphore,NULL );
			active_menu = Auto_Wash_Menu;
			xSemaphoreGiveFromISR ( lcd_semaphore,NULL );
			HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_1);  
			__HAL_TIM_SET_COUNTER(&htim5, 0);
		}	
		else if(*auto_wash_state == 1)
		{
			HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
			relay_on_off(supply_func_num , 0);  
			relay_on_off(drain_func_num , 0);
			relay_on_off(kcl_func_num , 0);
			relay_on_off(wash_func_num , 0);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,0,NULL,-1,0,0);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,1,NULL,-1,0,0);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,2,NULL,-1,0,0);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,3,NULL,-1,0,0); 
			
			xSemaphoreGiveFromISR ( lcd_semaphore,NULL );
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, supply_func_time*2000);
			__HAL_TIM_SET_COUNTER(&htim5, 0);
			*auto_wash_state = 2;	
		}
		else if(*auto_wash_state == 2)
		{
			relay_on_off(supply_func_num , 0);  
			relay_on_off(drain_func_num , 1);
			relay_on_off(kcl_func_num , 0);
			relay_on_off(wash_func_num , 0); 
			Change_Menu_Items(AUTO_WASH_STATE_MENU,0,NULL,-1,0,1);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,1,NULL,-1,1,1);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,2,NULL,-1,0,1);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,3,NULL,-1,0,1); 
			xSemaphoreGiveFromISR ( lcd_semaphore,NULL );
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, drain_func_time*2000);
			__HAL_TIM_SET_COUNTER(&htim5, 0);
		*auto_wash_state = 3;	
		}
		else if(*auto_wash_state == 3)
		{
			relay_on_off(supply_func_num , 0);  
			relay_on_off(drain_func_num , 0);
			relay_on_off(kcl_func_num , 1);
			relay_on_off(wash_func_num , 0);
			AWS_Supply_State = 0;
			Change_Menu_Items(AUTO_WASH_STATE_MENU,0,NULL,-1,0,2);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,1,NULL,-1,0,2);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,2,NULL,-1,1,2);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,3,NULL,-1,0,2); 	
			xSemaphoreGiveFromISR ( lcd_semaphore,NULL );
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, kcl_func_time*2000);
			__HAL_TIM_SET_COUNTER(&htim5, 0);
			*auto_wash_state = 4;	
		}
		else if(*auto_wash_state == 4)
		{
			relay_on_off(supply_func_num , 0);  
			relay_on_off(drain_func_num , 0);
			relay_on_off(kcl_func_num , 0);
			relay_on_off(wash_func_num , 1); 
			Change_Menu_Items(AUTO_WASH_STATE_MENU,0,NULL,-1,0,3);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,1,NULL,-1,0,3);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,2,NULL,-1,0,3);
			Change_Menu_Items(AUTO_WASH_STATE_MENU,3,NULL,-1,1,3); 		
			xSemaphoreGiveFromISR ( lcd_semaphore,NULL );
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, wash_func_time*2000);
			__HAL_TIM_SET_COUNTER(&htim5, 0);
		*auto_wash_state = 0;	
		}
}
void run_auto_wash()
{
	auto_wash_state = 1;
	auto_wash_handler(&auto_wash_state);
}
void manual_wash_exit()
{
	//relay_on_off(supply_func_num , 1);  
	//relay_on_off(drain_func_num , 0);
	//relay_on_off(kcl_func_num , 0);
	//relay_on_off(wash_func_num , 0); 
}
	
