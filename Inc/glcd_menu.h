/**
  ******************************************************************************
  * @file    glcd_menu.h 
  * @brief   general menu for glcd.
  ******************************************************************************
	by sepehr hashtroudi
	sepehrhashtroudi@gmail.com
  ******************************************************************************
  */
	typedef struct Menu
	{
		int menu_id;
		char menu_name[20];
		int menu_pointer;
		int menu_item_count;
		char menu_items[8][20];
		int menu_items_id[8];
		float menu_values[8];
	} Menu;
		
	int init_menu();

