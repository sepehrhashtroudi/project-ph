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
	#include "defines.h"
	typedef struct Menu
	{
		int menu_id;
		char menu_name[25];
		int menu_pointer;
		int menu_item_count;
		char menu_strings[8][MENU_STRING_LENGTH];
		int next_menu_id[8];
		float values[8];
		float value_resolution[8];
		float value_max[8];
		int x_position[8];
		int y_position[8];
		int font[8];
		void (*fun_ptr)(void);
		int run_on_exit;
		
		
		
	} Menu;
		
void init_menu(void);
void print_menu(int active_menu);
void print_main_page(int active_menu);
void get_user_input(uint8_t *input,int *active_menu);
void update_menu_from_variables(int active_menu);
void auto_wash_handler(int *auto_wash_state);
void Change_Menu_Items(int Menu_num, int Menu_line, char* Menu_String, int Next_Menu_Id, float Value, int Menu_Active_Line); // if you dont want to change arg set it -1 or null for strings
