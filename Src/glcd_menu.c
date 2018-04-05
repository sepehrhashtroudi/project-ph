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
extern Menu menu_list[6];

int init_menu()
{
	strcpy(menu_list[0].menu_name , "Main menu");
	strcpy(menu_list[0].menu_items[0],  "Calibration ");
	strcpy(menu_list[0].menu_items[1] , "Controller  ");
	strcpy(menu_list[0].menu_items[2] , "Temp sensor ");
	strcpy(menu_list[0].menu_items[3] , "Date&time   ");
	menu_list[0].menu_id=0;
	menu_list[0].menu_item_count = 4;
	menu_list[0].menu_items_id[0]=1;
	menu_list[0].menu_items_id[1]=4;
	menu_list[0].menu_items_id[2]=5;
	menu_list[0].menu_pointer=3;
	
	strcpy(menu_list[1].menu_name , "Calibration wizard");
	strcpy(menu_list[1].menu_items[0],  "Step 1");
	strcpy(menu_list[1].menu_items[1] , "Enter buffer pH:%d");
	strcpy(menu_list[1].menu_items[2] , "%d");
	strcpy(menu_list[1].menu_items[3] , "ok");
	menu_list[1].menu_items_id[0]=1;
	menu_list[1].menu_items_id[1]=1;
	menu_list[1].menu_items_id[2]=1;
	menu_list[1].menu_items_id[3]=2;
	menu_list[1].menu_id=1;
	menu_list[1].menu_item_count = 4;
	menu_list[1].menu_pointer=1;
	
	strcpy(menu_list[2].menu_name , "Calibration wizard");
	strcpy(menu_list[2].menu_items[0],  "Step 2 ");
	strcpy(menu_list[2].menu_items[1] , "Enter second buffer pH:%d");
	strcpy(menu_list[2].menu_items[2] , "%d");
	strcpy(menu_list[2].menu_items[3] , "ok");
	menu_list[2].menu_items_id[0]=2;
	menu_list[2].menu_items_id[1]=2;
	menu_list[2].menu_items_id[2]=2;
	menu_list[2].menu_items_id[3]=3;
	menu_list[2].menu_id=2;
	menu_list[2].menu_item_count = 4;
	menu_list[2].menu_pointer=1;
	
	strcpy(menu_list[3].menu_name , "Calibration wizard");
	strcpy(menu_list[3].menu_items[0],  "Step 3 ");
	strcpy(menu_list[3].menu_items[1] , "Done");
	menu_list[3].menu_items_id[0]=0;
	menu_list[3].menu_items_id[1]=0;
	menu_list[3].menu_id=3;
	menu_list[3].menu_item_count = 2;
	menu_list[3].menu_pointer=1;
	
	strcpy(menu_list[4].menu_name , "Controller");
	strcpy(menu_list[4].menu_items[0],  "ON/OFF");
	strcpy(menu_list[4].menu_items[1] , "TYPE");
	menu_list[4].menu_id=4;
	menu_list[4].menu_item_count = 2;
	menu_list[4].menu_pointer=1;
	
}
void update_menu_from_variables()
{
	
}