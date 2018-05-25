/**
  ******************************************************************************
  * @file    glcd_menu.h 
  * @brief   general menu for glcd.
  ******************************************************************************
	by sepehr hashtroudi
	sepehrhashtroudi@gmail.com
  ******************************************************************************
  */
	#include "stm32f4xx_hal.h"
	typedef struct Menu
	{
		int menu_id;
		char menu_name[25];
		int menu_pointer;
		int menu_item_count;
		char menu_strings[8][25];
		int next_menu_id[8];
		float values[8];
		float value_resolution[8];
		float value_max[8];
		int x_position[8];
		int y_position[8];
		int font[8];
		void (*fun_ptr)(void);
		
		
	} Menu;
		
void init_menu(void);
void print_menu(int active_menu);
void print_main_page(int active_menu);
void get_user_input(uint8_t *input,int *active_menu);
void update_menu_from_variables(void);

	