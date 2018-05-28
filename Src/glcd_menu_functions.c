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
extern Menu menu_list[20];
extern float p1_p;
extern float p2_p;
extern float p1_t;
extern float p2_t;
int32_t EEprom_buff;
float *calibration_point_1_ph = &menu_list[2].values[1];
float *calibration_point_2_ph = &menu_list[4].values[1];
float *calibration_point_1_temp = &menu_list[12].values[1];
float *calibration_point_2_temp = &menu_list[14].values[1];
uint16_t calibration_point_1 = 0;
uint16_t calibration_point_2 = 0;
extern int create_ph_calibration_task_flag;
extern int delete_ph_calibration_task_flag;
extern int create_temp_calibration_task_flag;
extern int delete_temp_calibration_task_flag;
extern uint16_t pH_filtered;
extern uint16_t temp_filtered;
void ph_calculate_calibration_coefficients(void)
{
	char sprintf_buff[20];
	p1_p = (*calibration_point_1_ph - *calibration_point_2_ph) / (float)(calibration_point_1 - calibration_point_2 ) ;
	p2_p = *calibration_point_1_ph - (calibration_point_1)*(p1_p);
	EEprom_buff = p1_p*float_to_int_factor;
	eeprom_write_data(p1_p_eeprom_add,&EEprom_buff,1);
	EEprom_buff = p2_p*float_to_int_factor;
	eeprom_write_data(p2_p_eeprom_add,&EEprom_buff,1);
	sprintf(sprintf_buff,"%.5f,%.5f\n",p1_p,p2_p);
	MAX485_send_string(sprintf_buff,13,100);
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
}
void ph_calibration_waiting_2(void)
{
	delete_ph_calibration_task_flag = 1;
	calibration_point_2 = pH_filtered;
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

