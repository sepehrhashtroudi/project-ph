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
#include "font5x8.h"
#include "graphic.h"
extern Menu menu_list[6];


void init_menu(void)
{
	strcpy(menu_list[0].menu_name , "Main menu");
	strcpy(menu_list[0].menu_strings[0],  "Calibration");
	strcpy(menu_list[0].menu_strings[1] , "Controller");
	strcpy(menu_list[0].menu_strings[2] , "Temp sensor");
	strcpy(menu_list[0].menu_strings[3] , "Date&time");
	menu_list[0].menu_id=0;
	menu_list[0].menu_item_count = 4;
	menu_list[0].next_menu_id[0]=1;
	menu_list[0].next_menu_id[1]=4;
	menu_list[0].next_menu_id[2]=5;
	menu_list[0].menu_pointer=3;
	
	strcpy(menu_list[1].menu_name , "Calibration wizard");
	strcpy(menu_list[1].menu_strings[0],  "Step 1");
	strcpy(menu_list[1].menu_strings[1] , "buffer ph:%.1f");
	strcpy(menu_list[1].menu_strings[2] , "%d");
	strcpy(menu_list[1].menu_strings[3] , "ok");
	menu_list[1].next_menu_id[0]=1;
	menu_list[1].next_menu_id[1]=1;
	menu_list[1].next_menu_id[2]=1;
	menu_list[1].next_menu_id[3]=2;
	menu_list[1].values[1]=7.234;
	menu_list[1].values[2]=200;
	menu_list[1].menu_id=1;
	menu_list[1].menu_item_count = 4;
	menu_list[1].menu_pointer=2;
	
	strcpy(menu_list[2].menu_name , "Calibration wizard");
	strcpy(menu_list[2].menu_strings[0],  "Step 2 ");
	strcpy(menu_list[2].menu_strings[1] , "second buffer pH:%.1f");
	strcpy(menu_list[2].menu_strings[2] , "%d");
	strcpy(menu_list[2].menu_strings[3] , "ok");
	menu_list[2].next_menu_id[0]=2;
	menu_list[2].next_menu_id[1]=2;
	menu_list[2].next_menu_id[2]=2;
	menu_list[2].next_menu_id[3]=3;
	menu_list[2].menu_id=2;
	menu_list[2].menu_item_count = 4;
	menu_list[2].menu_pointer=1;
	
	strcpy(menu_list[3].menu_name , "Calibration wizard");
	strcpy(menu_list[3].menu_strings[0],  "Step 3 ");
	strcpy(menu_list[3].menu_strings[1] , "Done");
	menu_list[3].next_menu_id[0]=0;
	menu_list[3].next_menu_id[1]=0;
	menu_list[3].menu_id=3;
	menu_list[3].menu_item_count = 2;
	menu_list[3].menu_pointer=1;
	
	strcpy(menu_list[4].menu_name , "Controller");
	strcpy(menu_list[4].menu_strings[0], "ON,OFF");
	strcpy(menu_list[4].menu_strings[1], "PID,RELAY");
	strcpy(menu_list[4].menu_strings[2], "setpoint:%.1f");
	strcpy(menu_list[4].menu_strings[3], "coefficients,hysteresis");
	menu_list[4].next_menu_id[0]=4;
	menu_list[4].next_menu_id[1]=4;
	menu_list[4].next_menu_id[2]=4;
	menu_list[4].next_menu_id[3]=5;
	menu_list[4].values[0]=1;
	menu_list[4].values[1]=1;
	menu_list[4].values[2]=7.100;
	menu_list[4].values[3]=1;
	menu_list[4].value_resolution[0]=1;
	menu_list[4].value_resolution[1]=1;
	menu_list[4].value_resolution[2]=0.1;
	menu_list[4].value_resolution[3]=1;
	menu_list[4].value_max[0]=1;
	menu_list[4].value_max[1]=1;
	menu_list[4].value_max[2]=14;
	menu_list[4].value_max[3]=1;
	menu_list[4].menu_id=4;
	menu_list[4].menu_item_count = 4;
	menu_list[4].menu_pointer=1;
	
}
void update_menu_from_variables()
{
	
}
void print_menu(int active_menu)
{
		char *search_buff;
		int print_offset=0;
		GLCD_ClearScreen();
		glcd_set_font(Terminal6x8 ,6,8,32,127);
		
		print_offset = (128 - CalcTextWidthEN( menu_list[active_menu].menu_name))/2;
		glcd_draw_string_xy(print_offset,0,menu_list[active_menu].menu_name,0,0,0);
		GLCD_Line(10,10,117,10);
		for(int i=0 ; i < menu_list[active_menu].menu_item_count ; i++)
		{
			char Menu_strings_buff[22];
			char *menu_strings_buff = Menu_strings_buff;
			char Final_menu_strings[22];
			char *final_menu_strings = Final_menu_strings;
			
			strcpy(menu_strings_buff , menu_list[active_menu].menu_strings[i]);
			if (strstr(menu_strings_buff, "%d") != 0)
			{
				sprintf(final_menu_strings, menu_strings_buff , (int)(menu_list[active_menu].values[i]));
			}
			else if (strstr(menu_strings_buff, "%.1f") != 0)
			{
				sprintf(final_menu_strings, menu_strings_buff , (menu_list[active_menu].values[i]));
			}
			else if(strstr(menu_strings_buff, ",") != 0)
			{			
				final_menu_strings = strtok(menu_strings_buff , ",");
				for(int j=0 ; j<menu_list[active_menu].values[i] ; j++)
				{
					final_menu_strings = strtok(NULL , ",");
				}
			}
			else
			{
				strcpy(final_menu_strings , menu_list[active_menu].menu_strings[i]);
			}


			print_offset = (128 - CalcTextWidthEN( final_menu_strings))/2;
			if(i == menu_list[active_menu].menu_pointer)
			{
				glcd_draw_string_xy(print_offset,i*10+14,final_menu_strings,0,1,0);
			}
			else
			{ 
				glcd_draw_string_xy(print_offset,i*10+14,final_menu_strings,0,0,0);
			}
		}
	
}
void get_user_input(uint8_t *input,int *active_menu)
{
	if(input[0]=='\n')
	{
		*active_menu = menu_list[*active_menu].next_menu_id[menu_list[*active_menu].menu_pointer];
	}
	if(input[0] == 'w')
	{
		if(menu_list[*active_menu].menu_pointer > 0)
		{
		menu_list[*active_menu].menu_pointer--;
		}
		else
		{
			menu_list[*active_menu].menu_pointer = menu_list[*active_menu].menu_item_count-1;
		}
	}
	if(input[0] == 's')
	{
		if(menu_list[*active_menu].menu_pointer < menu_list[*active_menu].menu_item_count-1)
		{
		menu_list[*active_menu].menu_pointer++;
		}
		else
		{
			menu_list[*active_menu].menu_pointer = 0;
		}
	}
	if(input[0] == 'd')
	{
		if(menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] < menu_list[*active_menu].value_max[menu_list[*active_menu].menu_pointer])
		{
			menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] += menu_list[*active_menu].value_resolution[menu_list[*active_menu].menu_pointer] ;
		}
		else
		{
			menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] = 0;
		}
	}
	if(input[0] == 'a')
	{
		if(menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] > 0.001)
		{
			menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] -= menu_list[*active_menu].value_resolution[menu_list[*active_menu].menu_pointer] ;
		}
		else
		{
			menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] = menu_list[*active_menu].value_max[menu_list[*active_menu].menu_pointer];
		}
	}
	
	
}