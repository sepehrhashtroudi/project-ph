/**
  ******************************************************************************
  * @file    glcd_menu.c 
  * @brief   general menu for glcd.
  ******************************************************************************
	by sepehr hashtroudi
	sepehrhashtroudi@gmail.com
  ******************************************************************************
  */
#include "glcd_menu.h"
#include <string.h>
#include "KS0108.h"
#include <string.h>
#include "usart.h"
//#include "font5x8.h"
#include "graphic.h"
#include <stdlib.h>
#include "max485.h"
#include "glcd_menu_functions.h"
#include "defines.h"
extern Menu menu_list[menu_list_length];
extern void set_date_time(void);
extern uint16_t pH_filtered;
extern uint16_t temp_filtered;
extern void set_pid_coefficients(void);
extern float output_mA;
extern int16_t pump_on_off_state;
extern int16_t progress;
extern int create_ph_calibration_task_flag;
extern int delete_ph_calibration_task_flag;
extern int create_temp_calibration_task_flag;
extern int delete_temp_calibration_task_flag;
extern int back_light_state;
extern float pH;
extern float temp;
extern int active_menu_sp;
extern int auto_wash_state;
extern int back_button_flag;
extern int ok_button_flag;
extern int up_button_flag;
extern int down_button_flag; 
extern int left_button_flag;
extern int right_button_flag;
extern int please_wait_flag;

void init_menu(void)
{
	strcpy(menu_list[0].menu_name , "first page");
	strcpy(menu_list[0].menu_strings[0],  "%.2f");
	strcpy(menu_list[0].menu_strings[1] , "pH");
	strcpy(menu_list[0].menu_strings[2] , "%.1f `C");
	strcpy(menu_list[0].menu_strings[3] , "%02d:%02d");//hour & minute
	strcpy(menu_list[0].menu_strings[5] , "%.1f mA");
	//strcpy(menu_list[0].menu_strings[5] , "%d"); //minute
	menu_list[0].menu_id=0;
	menu_list[0].menu_item_count = 6;
	menu_list[0].menu_pointer=0;
	menu_list[0].values[0]=7;
	menu_list[0].values[3]=10;
	menu_list[0].x_position[0]=64;
	menu_list[0].x_position[1]=92;
	menu_list[0].x_position[2]=64;
	menu_list[0].x_position[3]=16;
	menu_list[0].x_position[4]=16;
	menu_list[0].x_position[5]=105;
	menu_list[0].y_position[0]=4;
	menu_list[0].y_position[1]=15;
	menu_list[0].y_position[2]=35;
	menu_list[0].y_position[3]=52;
	menu_list[0].y_position[4]=52;
	menu_list[0].y_position[5]=52;
	menu_list[0].font[0]=1;
	menu_list[0].font[1]=0;
	menu_list[0].font[2]=0;
	menu_list[0].font[3]=0;
	menu_list[0].font[4]=0;
	menu_list[0].font[5]=0;
	menu_list[0].run_on_exit=0;
	
	strcpy(menu_list[1].menu_name , "Main menu");
	strcpy(menu_list[1].menu_strings[0],  " Calibration ");
	strcpy(menu_list[1].menu_strings[1] , " Controller ");
	strcpy(menu_list[1].menu_strings[2] , " Measurement ");
	strcpy(menu_list[1].menu_strings[3] , " Self Cleaning ");
	strcpy(menu_list[1].menu_strings[4] , " Time ");
	menu_list[1].menu_id=1;
	menu_list[1].menu_item_count = 5;
	menu_list[1].next_menu_id[0]=7;
	menu_list[1].next_menu_id[1]=8;
	menu_list[1].next_menu_id[2]=22;
	menu_list[1].next_menu_id[3]=19;
	menu_list[1].next_menu_id[4]=11;
	menu_list[1].menu_pointer=0;
	menu_list[1].run_on_exit=0;
	
	strcpy(menu_list[2].menu_name , " Step 1 ");
	strcpy(menu_list[2].menu_strings[0] , "< Buffer pH: %.1f >");
	strcpy(menu_list[2].menu_strings[1] , "< Stab Time: %d s >");
	strcpy(menu_list[2].menu_strings[2] , "< Stab Range: %.2f pH >");
	strcpy(menu_list[2].menu_strings[3] , " OK ");
	menu_list[2].next_menu_id[0]=2;
	menu_list[2].next_menu_id[1]=2;
	menu_list[2].next_menu_id[2]=2;
	menu_list[2].next_menu_id[3]=3;
	menu_list[2].values[0]= 4.000;
	menu_list[2].values[1]= 10;
	menu_list[2].values[2]= 0.03;
	menu_list[2].value_resolution[0] = 0.1000;
	menu_list[2].value_resolution[1] = 5;
	menu_list[2].value_resolution[2] = 0.01;
	menu_list[2].value_max[0] = 14;
	menu_list[2].value_max[1] = 30;
	menu_list[2].value_min[1] = 5;
	menu_list[2].value_max[2] = 0.2;
	menu_list[2].value_min[2] = 0.02;
	menu_list[2].menu_id=2;
	menu_list[2].menu_item_count = 4;
	menu_list[2].menu_pointer=0;
	menu_list[2].fun_ptr = &ph_calibration_step1;
	menu_list[2].run_on_exit=0;
	
	strcpy(menu_list[3].menu_name , " Step 1 ");
	strcpy(menu_list[3].menu_strings[0],  " %d mV ");
	strcpy(menu_list[3].menu_strings[1] , " %.2f pH ");
	strcpy(menu_list[3].menu_strings[2] , " %.1f `C ");
	strcpy(menu_list[3].menu_strings[3] , " Please Wait ");
	menu_list[3].next_menu_id[0]=3;
	menu_list[3].next_menu_id[1]=3;
	menu_list[3].next_menu_id[2]=3;
	menu_list[3].menu_id=3;
	menu_list[3].values[0] =0;
	menu_list[3].values[1] =0;
	menu_list[3].values[2] =0;
	menu_list[3].values[3] =0;
	menu_list[3].menu_item_count = 4;
	menu_list[3].menu_pointer=1;
	menu_list[3].fun_ptr = &ph_calibration_waiting_1;
	menu_list[3].run_on_exit=0;
	
	strcpy(menu_list[4].menu_name , " Step 2 ");
	strcpy(menu_list[4].menu_strings[0] , "< Buffer pH: %.1f >");
	strcpy(menu_list[4].menu_strings[1] , " OK ");
	menu_list[4].next_menu_id[0]=4;
	menu_list[4].next_menu_id[1]=5;
	menu_list[4].menu_id=4;
	menu_list[4].values[0]=7.000;
	menu_list[4].value_resolution[0]=0.1000;
	menu_list[4].value_max[0]=14;
	menu_list[4].menu_item_count = 2;
	menu_list[4].menu_pointer=0;
	menu_list[4].fun_ptr = &ph_calibration_step1;
	menu_list[4].run_on_exit=0;
	
	strcpy(menu_list[5].menu_name , " Step 2 ");
	strcpy(menu_list[5].menu_strings[0],  " %d mV ");
	strcpy(menu_list[5].menu_strings[1] , " %.2f pH ");
	strcpy(menu_list[5].menu_strings[2] , " %.1f `C ");
	strcpy(menu_list[5].menu_strings[3] , " Please Wait ");
	menu_list[5].next_menu_id[0]=5;
	menu_list[5].next_menu_id[1]=5;
	menu_list[5].next_menu_id[2]=5;
	menu_list[5].menu_id=5;
	menu_list[5].values[0] =0;
	menu_list[5].values[1] =0;
	menu_list[5].values[2] =0;
	menu_list[5].values[3] =0;
	menu_list[5].menu_item_count = 4;
	menu_list[5].menu_pointer=1;
	menu_list[5].fun_ptr = &ph_calibration_waiting_2;
	menu_list[5].run_on_exit=0;
	
	strcpy(menu_list[6].menu_name , " Done ");
	strcpy(menu_list[6].menu_strings[0], " Slope: %d %% ");
	strcpy(menu_list[6].menu_strings[1], " Zero: %d mV ");
	strcpy(menu_list[6].menu_strings[2], "Sensor is healthy");
	strcpy(menu_list[6].menu_strings[3] , " OK ");
	menu_list[6].next_menu_id[0]=6;
	menu_list[6].next_menu_id[1]=6;
	menu_list[6].next_menu_id[2]=6;
	menu_list[6].next_menu_id[3]=0;
	menu_list[6].menu_id=6;
	menu_list[6].menu_item_count = 4;
	menu_list[6].menu_pointer=1;
	menu_list[6].fun_ptr = &ph_calculate_calibration_coefficients;
	menu_list[6].run_on_exit=0;
	
	strcpy(menu_list[7].menu_name , " Select Sensor");
	strcpy(menu_list[7].menu_strings[0],  " pH ");
	strcpy(menu_list[7].menu_strings[1] , " Temp ");
	menu_list[7].next_menu_id[0]=2;
	menu_list[7].next_menu_id[1]=12;
	menu_list[7].menu_id=7;
	menu_list[7].menu_item_count = 2;
	menu_list[7].menu_pointer=0;
	menu_list[7].run_on_exit=0;
	
	strcpy(menu_list[8].menu_name , " Controller ");
	strcpy(menu_list[8].menu_strings[0], " < ON > , < OFF >");
	strcpy(menu_list[8].menu_strings[1], " < PID > , < RELAY > ");
	strcpy(menu_list[8].menu_strings[2], " Coefficients , Hysteresis ");
	strcpy(menu_list[8].menu_strings[3], " Setpoint: %.1f ");
	menu_list[8].next_menu_id[0]=8;
	menu_list[8].next_menu_id[1]=8;
	menu_list[8].next_menu_id[2]=9;
	menu_list[8].next_menu_id[3]=8;
	menu_list[8].values[0]=0;
	menu_list[8].values[1]=0;
	menu_list[8].values[2]=0;
	menu_list[8].values[3]=7.1;
	menu_list[8].value_resolution[0]=1.00000;
	menu_list[8].value_resolution[1]=1.00000;
	menu_list[8].value_resolution[2]=1.00000;
	menu_list[8].value_resolution[3]=0.10000;
	menu_list[8].value_max[0]=1;
	menu_list[8].value_max[1]=1;
	menu_list[8].value_max[2]=1;
	menu_list[8].value_max[3]=14;
	menu_list[8].menu_id=8;
	menu_list[8].menu_item_count = 4;
	menu_list[8].menu_pointer=0;
	menu_list[8].fun_ptr = &set_controller_set_point;
	menu_list[8].run_on_exit=1;

	
	
	strcpy(menu_list[9].menu_name , "Hysteresis");
	strcpy(menu_list[9].menu_strings[0], "< Max: %.1f >");
	strcpy(menu_list[9].menu_strings[1], "< Min: %.1f >");
	menu_list[9].next_menu_id[0]=9;
	menu_list[9].next_menu_id[1]=9;
	menu_list[9].next_menu_id[2]=0;
	menu_list[9].values[0]=8;
	menu_list[9].values[1]=6;
	menu_list[9].value_resolution[0]=0.10000;
	menu_list[9].value_resolution[1]=0.10000;
	menu_list[9].value_max[0]=14;
	menu_list[9].value_max[1]=14;
	menu_list[9].menu_id=6;
	menu_list[9].menu_item_count = 2;
	menu_list[9].menu_pointer=0;
	menu_list[9].fun_ptr = &set_relay_Hysteresis;
	menu_list[9].run_on_exit=1;
	
	strcpy(menu_list[10].menu_name , "PID Coefficients");
	strcpy(menu_list[10].menu_strings[0], "< P: %d >");
	strcpy(menu_list[10].menu_strings[1], "< I: %.1f >");
	strcpy(menu_list[10].menu_strings[2], "< D: %.1f >");
	menu_list[10].next_menu_id[0]=10;
	menu_list[10].next_menu_id[1]=10;
	menu_list[10].next_menu_id[2]=10;
	menu_list[10].next_menu_id[3]=0;
	menu_list[10].values[0]=100;
	menu_list[10].values[1]=1;
	menu_list[10].values[2]=0;
	menu_list[10].value_resolution[0]=1;
	menu_list[10].value_resolution[1]=0.1000;
	menu_list[10].value_resolution[2]=0.1000;
	menu_list[10].value_max[0]=400;
	menu_list[10].value_max[1]=20;
	menu_list[10].value_max[2]=10;
	menu_list[10].menu_id=7;
	menu_list[10].menu_item_count = 3;
	menu_list[10].menu_pointer=0;
	menu_list[10].fun_ptr = &set_pid_coefficients;
	menu_list[10].run_on_exit=1;
	
	strcpy(menu_list[11].menu_name , "Time");
	strcpy(menu_list[11].menu_strings[0], "< Hour: %d >");
	strcpy(menu_list[11].menu_strings[1], "< Minute: %d >");
//	strcpy(menu_list[11].menu_strings[2], "< Day: %d >");
//	strcpy(menu_list[11].menu_strings[3], "< Month: %d >");
//	strcpy(menu_list[11].menu_strings[4], "< Year: %d >");
	strcpy(menu_list[11].menu_strings[2], " OK ");
	menu_list[11].next_menu_id[0]=11;
	menu_list[11].next_menu_id[1]=11;
	menu_list[11].next_menu_id[2]=1;
//	menu_list[11].next_menu_id[3]=11;
//	menu_list[11].next_menu_id[4]=11;
//	menu_list[11].next_menu_id[5]=1;
	menu_list[11].values[0]=12;
	menu_list[11].values[1]=30;
	menu_list[11].values[2]=15;
	menu_list[11].values[3]=6;
	menu_list[11].values[4]=2018;
	menu_list[11].value_resolution[0]=1.000;
	menu_list[11].value_resolution[1]=1.000;
	menu_list[11].value_resolution[2]=1.000;
	menu_list[11].value_resolution[3]=1.000;
	menu_list[11].value_resolution[4]=1.000;
	menu_list[11].value_max[0]=23;
	menu_list[11].value_max[1]=59;
	menu_list[11].value_max[1]=31;
	menu_list[11].value_max[1]=12;
	menu_list[11].value_max[1]=3000;
	menu_list[11].menu_id=11;
	menu_list[11].menu_item_count = 3;
	menu_list[11].menu_pointer=0;
	menu_list[11].fun_ptr = &set_date_time;
	menu_list[11].run_on_exit=0;
	
	strcpy(menu_list[12].menu_name , "Step 1");
	strcpy(menu_list[12].menu_strings[0] , "< Bath Temp: %d >");
	strcpy(menu_list[12].menu_strings[1] , " OK ");
	menu_list[12].next_menu_id[0]=12;
	menu_list[12].next_menu_id[1]=13;
	menu_list[12].values[0]=25;
	menu_list[12].value_resolution[0]=1.0000;
	menu_list[12].value_max[0]=100;
	menu_list[12].menu_id=12;
	menu_list[12].menu_item_count = 2;
	menu_list[12].menu_pointer=0;
	menu_list[12].fun_ptr = &temp_calibration_step1;
	menu_list[12].run_on_exit=0;
	
	strcpy(menu_list[13].menu_name , " Step 1 ");
	strcpy(menu_list[13].menu_strings[0],  " %d mV ");
	strcpy(menu_list[13].menu_strings[1] , " %.1f `C ");
	strcpy(menu_list[13].menu_strings[2] , " Please Wait ");
	menu_list[13].next_menu_id[0]=13;
	menu_list[13].next_menu_id[1]=13;
	menu_list[13].next_menu_id[2]=13;
	menu_list[13].menu_id=13;
	menu_list[13].menu_item_count = 3;
	menu_list[13].menu_pointer=1;
	menu_list[13].fun_ptr = &temp_calibration_waiting_1;
	menu_list[13].run_on_exit=0;
	
	strcpy(menu_list[14].menu_name , " Step 2 ");
	strcpy(menu_list[14].menu_strings[0] , "< Bath Temp: %d >");
	strcpy(menu_list[14].menu_strings[1] , " OK ");
	menu_list[14].next_menu_id[0]=14;
	menu_list[14].next_menu_id[1]=15;
	menu_list[14].menu_id=14;
	menu_list[14].values[0]=35;
	menu_list[14].value_resolution[0]=1.0000;
	menu_list[14].value_max[0]=100;
	menu_list[14].menu_item_count = 2;
	menu_list[14].menu_pointer=0;
	menu_list[14].fun_ptr = &temp_calibration_step2;
	menu_list[14].run_on_exit=0;
	
	strcpy(menu_list[15].menu_name , " Step 2 ");
	strcpy(menu_list[15].menu_strings[0],  " %d mV ");
	strcpy(menu_list[15].menu_strings[1] , " %.1f `C ");
	strcpy(menu_list[15].menu_strings[2] , " Please Wait ");
	menu_list[15].next_menu_id[0]=15;
	menu_list[15].next_menu_id[1]=15;
	menu_list[15].next_menu_id[2]=15;
	menu_list[15].menu_id=15;
	menu_list[15].menu_item_count = 3;
	menu_list[15].menu_pointer=1;
	menu_list[15].fun_ptr = &temp_calibration_waiting_2;
	menu_list[15].run_on_exit=0;
	
	strcpy(menu_list[16].menu_name , " Step 3 ");
	strcpy(menu_list[16].menu_strings[0],  " Calibration Done ");
	strcpy(menu_list[16].menu_strings[1] , " OK ");
	menu_list[16].next_menu_id[0]=16;
	menu_list[16].next_menu_id[1]=0;
	menu_list[16].menu_id=16;
	menu_list[16].menu_item_count = 2;
	menu_list[16].menu_pointer=1;
	menu_list[16].fun_ptr = &temp_calculate_calibration_coefficients;
	menu_list[16].run_on_exit=1;
	
	strcpy(menu_list[17].menu_name , "Relay Functions");
	strcpy(menu_list[17].menu_strings[0], " K1:| <Pump>,<Supply>,<Drain>,<Wash>,<KCL>");
	strcpy(menu_list[17].menu_strings[1], " K2:| <Pump>,<Supply>,<Drain>,<Wash>,<KCL>");
	strcpy(menu_list[17].menu_strings[2], " K3:| <Pump>,<Supply>,<Drain>,<Wash>,<KCL>");
	strcpy(menu_list[17].menu_strings[3], " K4:| <Pump>,<Supply>,<Drain>,<Wash>,<KCL>");
	menu_list[17].next_menu_id[0]=17;
	menu_list[17].next_menu_id[1]=17;
	menu_list[17].next_menu_id[2]=17;
	menu_list[17].next_menu_id[3]=17;
	menu_list[17].values[0]=0;
	menu_list[17].values[1]=1;
	menu_list[17].values[2]=2;
	menu_list[17].values[3]=3;
	menu_list[17].value_resolution[0]=1;
	menu_list[17].value_resolution[1]=1;
	menu_list[17].value_resolution[2]=1;
	menu_list[17].value_resolution[3]=1;
	menu_list[17].value_max[0]=4;
	menu_list[17].value_max[1]=4;
	menu_list[17].value_max[2]=4;
	menu_list[17].value_max[3]=4;
	menu_list[17].menu_id=17;
	menu_list[17].menu_item_count = 4;
	menu_list[17].menu_pointer=0;
	menu_list[17].fun_ptr = &relay_func_exit;
	menu_list[17].run_on_exit=1;
	
	
	strcpy(menu_list[18].menu_name , "Manual Wash");
	strcpy(menu_list[18].menu_strings[0], " Supply:| <OFF>,<ON>");
	strcpy(menu_list[18].menu_strings[1], " Drain:| <OFF>,<ON>");
	strcpy(menu_list[18].menu_strings[2], " Wash:| <OFF>,<ON>");
	strcpy(menu_list[18].menu_strings[3], " KCL:| <OFF>,<ON>");
	menu_list[18].next_menu_id[0]=18;
	menu_list[18].next_menu_id[1]=18;
	menu_list[18].next_menu_id[2]=18;
	menu_list[18].next_menu_id[3]=18;
	menu_list[18].values[0]=0;
	menu_list[18].values[1]=0;
	menu_list[18].values[2]=0;
	menu_list[18].values[3]=0;
	menu_list[18].value_resolution[0]=1;
	menu_list[18].value_resolution[1]=1;
	menu_list[18].value_resolution[2]=1;
	menu_list[18].value_resolution[3]=1;
	menu_list[18].value_max[0]=1;
	menu_list[18].value_max[1]=1;
	menu_list[18].value_max[2]=1;
	menu_list[18].value_max[3]=1;
	menu_list[18].menu_id=18;
	menu_list[18].menu_item_count = 4;
	menu_list[18].menu_pointer=0;
	menu_list[18].fun_ptr = &manual_wash_exit;
	menu_list[18].run_on_exit=1;
	
	strcpy(menu_list[19].menu_name , "Self Cleaning");
	strcpy(menu_list[19].menu_strings[0], " Relay Functions ");
	strcpy(menu_list[19].menu_strings[1], " Auto Wash ");
	strcpy(menu_list[19].menu_strings[2], " Manual Wash ");
	menu_list[19].next_menu_id[0]=17;
	menu_list[19].next_menu_id[1]=20;
	menu_list[19].next_menu_id[2]=18;
	menu_list[19].values[0]=0;
	menu_list[19].values[1]=0;
	menu_list[19].values[2]=0;
	menu_list[19].value_resolution[0]=0;
	menu_list[19].value_resolution[1]=0;
	menu_list[19].value_resolution[2]=0;
	menu_list[19].menu_id=19;
	menu_list[19].menu_item_count = 3;
	menu_list[19].menu_pointer=0;
	menu_list[19].fun_ptr = NULL;
	menu_list[19].run_on_exit=0;
	
	strcpy(menu_list[20].menu_name , "Auto Wash");
	strcpy(menu_list[20].menu_strings[0], " Drain 1: %d ");
	strcpy(menu_list[20].menu_strings[1], " Wash: %d");
	strcpy(menu_list[20].menu_strings[2], " Drain 2: %d");
	strcpy(menu_list[20].menu_strings[3], " KCL: %d");
	strcpy(menu_list[20].menu_strings[4], " OK ");
	menu_list[20].next_menu_id[0]=20;
	menu_list[20].next_menu_id[1]=20;
	menu_list[20].next_menu_id[2]=20;
	menu_list[20].next_menu_id[3]=20;
	menu_list[20].next_menu_id[4]=21;
	menu_list[20].values[0]=5;
	menu_list[20].values[1]=5;
	menu_list[20].values[2]=5;
	menu_list[20].values[3]=5;
	menu_list[20].value_resolution[0]= 5;
	menu_list[20].value_resolution[1]= 5;
	menu_list[20].value_resolution[2]= 5;
	menu_list[20].value_resolution[3]= 5;
	menu_list[20].value_max[0]=60;
	menu_list[20].value_max[1]=60;
	menu_list[20].value_max[2]=60;
	menu_list[20].value_max[3]=60;
	menu_list[20].menu_id=20;
	menu_list[20].menu_item_count = 5;
	menu_list[20].menu_pointer=0;
	menu_list[20].fun_ptr = &run_auto_wash;
	menu_list[20].run_on_exit=0;
	
	strcpy(menu_list[21].menu_name , "Auto Wash State");
	strcpy(menu_list[21].menu_strings[0], " Supply:| <ON>,<OFF>");
	strcpy(menu_list[21].menu_strings[1], " Drain:| <OFF>,<ON>");
	strcpy(menu_list[21].menu_strings[2], " Wash:| <OFF>,<ON>");
	strcpy(menu_list[21].menu_strings[3], " KCL:| <OFF>,<ON>");
	menu_list[21].next_menu_id[0]=21;
	menu_list[21].next_menu_id[1]=21;
	menu_list[21].next_menu_id[2]=21;
	menu_list[21].next_menu_id[3]=21;
	menu_list[21].values[0]=0;
	menu_list[21].values[1]=0;
	menu_list[21].values[2]=0;
	menu_list[21].values[3]=0;
	menu_list[21].value_resolution[0]=1;
	menu_list[21].value_resolution[1]=1;
	menu_list[21].value_resolution[2]=1;
	menu_list[21].value_resolution[3]=1;
	menu_list[21].value_max[0]=1;
	menu_list[21].value_max[1]=1;
	menu_list[21].value_max[2]=1;
	menu_list[21].value_max[3]=1;
	menu_list[21].menu_id=21;
	menu_list[21].menu_item_count = 4;
	menu_list[21].menu_pointer=0;
	menu_list[21].fun_ptr = NULL ;
	menu_list[21].run_on_exit=0;
	
	strcpy(menu_list[22].menu_name , "Measurement");
	strcpy(menu_list[22].menu_strings[0], " ATC:| <OFF>,<ON>");
	strcpy(menu_list[22].menu_strings[1], " Slope: %d %% ");
	strcpy(menu_list[22].menu_strings[2], " Zero: %d mV ");
	menu_list[22].next_menu_id[0]=22;
	menu_list[22].next_menu_id[1]=22;
	menu_list[22].next_menu_id[2]=22;
	menu_list[22].values[0]=0;
	menu_list[22].value_resolution[0]=1;
	menu_list[22].value_max[0]=1;
	menu_list[22].menu_id=22;
	menu_list[22].menu_item_count = 3;
	menu_list[22].menu_pointer=0;
	menu_list[22].fun_ptr = &Measurement_exit ;
	menu_list[22].run_on_exit=1;
}
void update_menu_from_variables(int active_menu)
{
	// pid,relay link to coeff and hysteresis
	menu_list[8].values[2]= CONTROLLER_TYPE;
	if(CONTROLLER_TYPE == 1)
	{
		Change_Menu_Items(8,2,NULL,9,-1,-1);
		menu_list[8].menu_item_count = 3;
	}
	else
	{
		Change_Menu_Items(8,2,NULL,10,-1,-1);
		menu_list[8].menu_item_count = 4;
	}
	//relay max min restriction
	if(menu_list[9].values[1] > menu_list[9].values[0])
	{
		menu_list[9].values[1] = menu_list[9].values[0];
	}
	
	Change_Menu_Items(3, 0, NULL, -1, -1000 * (pH_filtered - 2047) / 4095.0 , -1); //calculate sensor mv
	Change_Menu_Items(5, 0, NULL, -1, -1000 * (pH_filtered - 2047) / 4095.0 , -1);
	if((pH_filtered * 0.00413 - 1.454) > 14)
	{
		Change_Menu_Items(3, 1, NULL, -1,14 , -1);
		Change_Menu_Items(5, 1, NULL, -1,14, -1);
	}
	else if((pH_filtered * 0.00413 - 1.454) < 0)
	{
		Change_Menu_Items(3, 1, NULL, -1, 0, -1); // calculate ph with ideal coeff
		Change_Menu_Items(5, 1, NULL, -1, 0, -1);
	}
	else
	{
		Change_Menu_Items(3, 1, NULL, -1, pH_filtered * 0.00413 - 1.454, -1); // calculate ph with ideal coeff
		Change_Menu_Items(5, 1, NULL, -1, pH_filtered * 0.00413 - 1.454, -1);
	}
	
	Change_Menu_Items(13, 0, NULL, -1, temp_filtered, -1);
	Change_Menu_Items(13, 1, NULL, -1, temp, -1);
	Change_Menu_Items(15, 0, NULL, -1, temp_filtered, -1);
	Change_Menu_Items(15, 1, NULL, -1, temp, -1);
	
	
	if( progress >= 100 )
	{
		Change_Menu_Items(3, 3, " OK ", 4, -1, -1);
		Change_Menu_Items(5, 3, " OK ", 6, -1, -1);
		Change_Menu_Items(13, 2, " OK ",14, -1, -1);
		Change_Menu_Items(15, 2, " OK ",16, -1, -1);	
		back_light_state = 0;	//set backlight to 100%
	}
	else
	{
		Change_Menu_Items(3, 3, " Wait to stabilize ", 3, -1, -1);
		Change_Menu_Items(5, 3, " Wait to stabilize ", 5, -1, -1);
		Change_Menu_Items(13, 2, " Wait to stabilize ",13, -1, -1);
		Change_Menu_Items(15, 2, " Wait to stabilize ",15, -1, -1);		
	}
	
	if(active_menu == manual_wash_menu)
	{
		relay_on_off(supply_func_num,supply_relay_state);
		relay_on_off(drain_func_num,drain_relay_state);
		relay_on_off(kcl_func_num,kcl_relay_state);
		relay_on_off(wash_func_num,wash_relay_state);
	}
	

	if(CONTROLLER_ON_OFF == 0 && CONTROLLER_TYPE ==0)
	{
		Change_Menu_Items(0,5,"%.1f mA",-1,-1,-1);
		menu_list[0].x_position[5]=105;
	}
	else if(CONTROLLER_ON_OFF == 1 || CONTROLLER_TYPE==1)
	{
		Change_Menu_Items(0,5," ",-1,-1,-1);
	}
	
	if( temp_filtered < 10)
	{
		Change_Menu_Items(0,2,"NC `C",-1,-1,-1);
		Change_Menu_Items(3, 2,"NC `C", -1, temp, -1);
		Change_Menu_Items(5, 2,"NC `C", -1, temp, -1);
	}
	else
	{
		Change_Menu_Items(0, 2,"%.1f `C", -1, temp,-1);
		Change_Menu_Items(3, 2,"%.1f `C", -1, temp, -1);
		Change_Menu_Items(5, 2,"%.1f `C", -1, temp, -1);
	}
	
		
	menu_list[0].values[5] = output_mA; 
	menu_list[0].values[0] = pH;

	if(pH < 10)
	{
		menu_list[0].x_position[1]=95;
	}
	else
	{
		menu_list[0].x_position[1]=102;
	}
	
}


void print_main_page(int active_menu)
{
		int text_width=0;
		GLCD_Clear_Ram();
		char menu_strings_buff[MENU_STRING_LENGTH];
		char Final_menu_strings[MENU_STRING_LENGTH];
		for(int i=0 ; i < menu_list[active_menu].menu_item_count ; i++)
		{
			
			if(menu_list[active_menu].font[i] == 1)
			{
					glcd_set_font_with_num(1);
			}
			if(menu_list[active_menu].font[i] == 0)
			{
					glcd_set_font_with_num(0);
			}	
			strcpy(menu_strings_buff , menu_list[active_menu].menu_strings[i]);
			if (strstr(menu_strings_buff, "%d") != 0)
			{
				int count =0;
				const char *tmp = menu_strings_buff;
				while(strstr(tmp,"%d")!= 0) // fined number of %d in string
				{
					 tmp = strstr(tmp,"%d");
					 count++;
					 tmp++;
				}
				
				if(count == 1)
				{
					sprintf(Final_menu_strings, menu_strings_buff , (int)(menu_list[active_menu].values[i]));
				}
				if(count == 2)
				{
					sprintf(Final_menu_strings, menu_strings_buff , (int)(menu_list[active_menu].values[i]),(int)(menu_list[active_menu].values[i+1]));
					i++;
				}
			}
			else if (strstr(menu_strings_buff, "%02d") != 0)
			{
				int count =0;
				const char *tmp = menu_strings_buff;
				while(strstr(tmp,"%02d")!= 0) // fined number of %d in string
				{
					 tmp = strstr(tmp,"%02d");
					 count++;
					 tmp++;
				}
				
				if(count == 1)
				{
					sprintf(Final_menu_strings, menu_strings_buff , (int)(menu_list[active_menu].values[i]));
				}
				if(count == 2)
				{
					sprintf(Final_menu_strings, menu_strings_buff , (int)(menu_list[active_menu].values[i]),(int)(menu_list[active_menu].values[i+1]));
					i++;
				}
			}
			else if (strstr(menu_strings_buff, "%.1f") != 0)
			{
				sprintf(Final_menu_strings, menu_strings_buff , (menu_list[active_menu].values[i]));
			}
			else if (strstr(menu_strings_buff, "%.2f") != 0)
			{
				sprintf(Final_menu_strings, menu_strings_buff , (menu_list[active_menu].values[i]));
			}
			else if(strstr(menu_strings_buff, ",") != 0)
			{	
				char dummy[MENU_STRING_LENGTH];
				char *final_buff = dummy;
				final_buff = strtok(menu_strings_buff , ",");
				for(int j=0 ; j<(int)(menu_list[active_menu].values[i]) ; j++)
				{
					final_buff = strtok(NULL , ",");
				}
				strcpy(Final_menu_strings , final_buff);
			}
			else
			{
				strcpy(Final_menu_strings , menu_list[active_menu].menu_strings[i]);
			}
			text_width = CalcTextWidthEN(Final_menu_strings);
			glcd_draw_string_xy_with_ram(menu_list[active_menu].x_position[i] - (text_width/2),menu_list[active_menu].y_position[i],Final_menu_strings,0,0,0);
		}
		GLCD_Write_Ram();
}

void print_menu(int active_menu)
{
		int print_offset=0;
		char menu_strings_buff[MENU_STRING_LENGTH];
		char final_menu_strings[MENU_STRING_LENGTH];
		glcd_set_font_with_num(0);
		GLCD_Clear_Ram();
		print_offset = (128 - CalcTextWidthEN( menu_list[active_menu].menu_name))/2;
		glcd_draw_string_xy_with_ram(print_offset,0,menu_list[active_menu].menu_name,0,0,0);
		GLCD_Line(10,10,117,10);
		for(int i=0 ; i < menu_list[active_menu].menu_item_count ; i++)
		{
			strcpy(menu_strings_buff , menu_list[active_menu].menu_strings[i]);
			if (strstr(menu_strings_buff, "%d") != 0)
			{
				sprintf(final_menu_strings, menu_strings_buff , (int)(menu_list[active_menu].values[i]));
			}
			else if (strstr(menu_strings_buff, "%.1f") != 0)
			{
				sprintf(final_menu_strings, menu_strings_buff , (menu_list[active_menu].values[i]));
			}
			else if (strstr(menu_strings_buff, "%.2f") != 0)
			{
				sprintf(final_menu_strings, menu_strings_buff , (menu_list[active_menu].values[i]));
			}
			else if(strstr(menu_strings_buff, "|") != 0)
			{			
				
				char chank1[MENU_STRING_LENGTH];
				char* Chank1 = chank1;
				char chank2[MENU_STRING_LENGTH];
				char* Chank2 = chank2;
				char chank3[MENU_STRING_LENGTH];
				char* Chank3 = chank3;
				Chank1 = strtok(menu_strings_buff, "|");
				Chank2 = strtok(NULL , "|");
				Chank3 = strtok(Chank2 , ",");
				
				for(int j=0 ; j<(int)(menu_list[active_menu].values[i]) ; j++)
				{
					Chank3 = strtok(NULL , ",");
				}
				sprintf(final_menu_strings, "%s%s",Chank1, Chank3);
			}
			else if(strstr(menu_strings_buff, ",") != 0)
			{			
				char dummy[MENU_STRING_LENGTH];
				char *final_buff = dummy;
				final_buff = strtok(menu_strings_buff , ",");
				for(int j=0 ; j<(int)(menu_list[active_menu].values[i]) ; j++)
				{
					final_buff = strtok(NULL , ",");
				}
				strcpy(final_menu_strings , final_buff);
			}
			else
			{
				strcpy(final_menu_strings , menu_list[active_menu].menu_strings[i]);
			}
			print_offset = (128 - CalcTextWidthEN( final_menu_strings))/2;
			if(i == menu_list[active_menu].menu_pointer)
			{
				
				glcd_draw_string_xy_with_ram(print_offset,i*9+14,final_menu_strings,0,1,0);
			}
			else
			{ 
				glcd_draw_string_xy_with_ram(print_offset,i*9+14,final_menu_strings,0,0,0);
			}
		}
	GLCD_Write_Ram();
}

void get_user_input(uint8_t *input,int *active_menu)
{
	
	if(input[0]=='\n')
	{
		ok_button_flag =1;
	}
	if(input[0] == 'e')
	{
		back_button_flag = 1;
		delete_ph_calibration_task_flag = 1;   //kill calibration thread
		delete_temp_calibration_task_flag = 1; //kill calibration thread
		auto_wash_state = 0;
		auto_wash_handler(&auto_wash_state);		//cancel auto wash
	}
	if(input[0] == 'w')
	{
		up_button_flag =1;
	}
	if(input[0] == 's')
	{
		down_button_flag =1;
	}
	if(input[0] == 'd')
	{
		right_button_flag =1;
	}
	if(input[0] == 'a')
	{
		left_button_flag =1;
	}	
	
			char sp_buff[20];
			sprintf(sp_buff,"sp:%d,ac:%d \n",active_menu_sp,active_menu[active_menu_sp]);
			MAX485_send_string(sp_buff,strlen(sp_buff),100);
}
void Change_Menu_Items(int Menu_num, int Menu_line, char* Menu_String, int Next_Menu_Id, float Value, int Menu_Active_Line)
{
	if (Menu_String != NULL)
	{
		strcpy(menu_list[Menu_num].menu_strings[Menu_line] , Menu_String);
	}
	if( Next_Menu_Id != -1)
	{
		menu_list[Menu_num].next_menu_id[Menu_line] = Next_Menu_Id;
	}
	if( Value != -1)
	{
		menu_list[Menu_num].values[Menu_line] = Value;
	}
	if( Menu_Active_Line != -1)
	{
		menu_list[Menu_num].menu_pointer = Menu_Active_Line;
	}
}
