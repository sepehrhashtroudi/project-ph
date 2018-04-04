/**
  ******************************************************************************
  * @file    glcd_menu.h 
  * @brief   general menu for glcd.
  ******************************************************************************
	by sepehr hashtroudi
	sepehrhashtroudi@gmail.com
  ******************************************************************************
  */
	#include <string.h>
	typedef struct Menu
	{
		char menu_name[20];
		int menu_pointer;
		int menu_item_count;
		char menu_items[8][20];
		int menu_is_active;
	} Menu;
		
	typedef struct Menu_list
	{
		Menu main_menu;
		Menu calibration_menu;
		Menu temp_sensor_selection_menu;
		Menu date_time_menu;
		Menu controller_menu;
		Menu threshold_menu;
	} Menu_list;
