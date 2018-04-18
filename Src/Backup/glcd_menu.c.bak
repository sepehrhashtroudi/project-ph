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
extern Menu menu_list[9];


void init_menu(void)
{
	strcpy(menu_list[0].menu_name , "first page");
	strcpy(menu_list[0].menu_strings[0],  "%.1f");
	strcpy(menu_list[0].menu_strings[1] , "ph");
	strcpy(menu_list[0].menu_strings[2] , "%.1f `c");
	strcpy(menu_list[0].menu_strings[3] , "%d s/m");
	menu_list[0].menu_id=0;
	menu_list[0].menu_item_count = 4;
	menu_list[0].next_menu_id[0]=2;
	menu_list[0].next_menu_id[1]=5;
	menu_list[0].next_menu_id[2]=8;
	menu_list[0].menu_pointer=0;
	menu_list[0].values[0]=7;
	menu_list[0].values[3]=10;
	menu_list[0].x_position[0]=45;
	menu_list[0].x_position[1]=80;
	menu_list[0].x_position[2]=40;
	menu_list[0].x_position[3]=80;
	menu_list[0].y_position[0]=4;
	menu_list[0].y_position[1]=20;
	menu_list[0].y_position[2]=50;
	menu_list[0].y_position[3]=50;
	menu_list[0].font[0]=1;
	menu_list[0].font[1]=0;
	menu_list[0].font[2]=0;
	menu_list[0].font[3]=0;
	
	strcpy(menu_list[1].menu_name , "Main menu");
	strcpy(menu_list[1].menu_strings[0],  " Calibration ");
	strcpy(menu_list[1].menu_strings[1] , " Controller ");
	strcpy(menu_list[1].menu_strings[2] , " Temp sensor ");
	strcpy(menu_list[1].menu_strings[3] , " Date&time ");
	strcpy(menu_list[1].menu_strings[4] , " Exit ");
	menu_list[1].menu_id=1;
	menu_list[1].menu_item_count = 5;
	menu_list[1].next_menu_id[0]=2;
	menu_list[1].next_menu_id[1]=5;
	menu_list[1].next_menu_id[2]=8;
	menu_list[1].next_menu_id[2]=0;
	menu_list[1].menu_pointer=0;
	
	strcpy(menu_list[2].menu_name , "Calibration wizard");
	strcpy(menu_list[2].menu_strings[0],  " Step 1 ");
	strcpy(menu_list[2].menu_strings[1] , " Enter Buffer ph:%.1f ");
	strcpy(menu_list[2].menu_strings[2] , " wait until this number ");
	strcpy(menu_list[2].menu_strings[3] , " stops changing: %d ");
	strcpy(menu_list[2].menu_strings[4] , " Ok ");
	menu_list[2].next_menu_id[0]=2;
	menu_list[2].next_menu_id[1]=2;
	menu_list[2].next_menu_id[2]=2;
	menu_list[2].next_menu_id[3]=2;
	menu_list[2].next_menu_id[4]=3;
	menu_list[2].values[1]=7.200;
	menu_list[2].values[3]=200;
	menu_list[2].value_resolution[0]=0;
	menu_list[2].value_resolution[1]=0.1000;
	menu_list[2].value_resolution[2]=0;
	menu_list[2].value_max[0]=0;
	menu_list[2].value_max[1]=14;
	menu_list[2].menu_id=2;
	menu_list[2].menu_item_count = 5;
	menu_list[2].menu_pointer=3;
	
	strcpy(menu_list[3].menu_name , "Calibration wizard");
	strcpy(menu_list[3].menu_strings[0],  " Step 2 ");
	strcpy(menu_list[3].menu_strings[1] , " Second buffer pH:%.1f ");
	strcpy(menu_list[3].menu_strings[2] , " %d ");
	strcpy(menu_list[3].menu_strings[3] , " Ok ");
	menu_list[3].next_menu_id[0]=3;
	menu_list[3].next_menu_id[1]=3;
	menu_list[3].next_menu_id[2]=3;
	menu_list[3].next_menu_id[3]=4;
	menu_list[3].menu_id=3;
	menu_list[3].menu_item_count = 4;
	menu_list[3].menu_pointer=3;
	
	strcpy(menu_list[4].menu_name , "Calibration wizard");
	strcpy(menu_list[4].menu_strings[0],  " Step 3 ");
	strcpy(menu_list[4].menu_strings[1] , " Done ");
	menu_list[4].next_menu_id[0]=1;
	menu_list[4].next_menu_id[1]=1;
	menu_list[4].menu_id=4;
	menu_list[4].menu_item_count = 2;
	menu_list[4].menu_pointer=1;
	
	strcpy(menu_list[5].menu_name , " Controller ");
	strcpy(menu_list[5].menu_strings[0], " ON , OFF ");
	strcpy(menu_list[5].menu_strings[1], " PID , RELAY ");
	strcpy(menu_list[5].menu_strings[2], " Setpoint:%.1f ");
	strcpy(menu_list[5].menu_strings[3], " Coefficients , Hysteresis ");
	menu_list[5].next_menu_id[0]=5;
	menu_list[5].next_menu_id[1]=5;
	menu_list[5].next_menu_id[2]=5;
	menu_list[5].next_menu_id[3]=6;
	menu_list[5].values[0]=1;
	menu_list[5].values[1]=1;
	menu_list[5].values[2]=7.100;
	menu_list[5].values[3]=1;
	menu_list[5].value_resolution[0]=1.00000;
	menu_list[5].value_resolution[1]=1.00000;
	menu_list[5].value_resolution[2]=0.10000;
	menu_list[5].value_resolution[3]=0.00000;
	menu_list[5].value_max[0]=1;
	menu_list[5].value_max[1]=1;
	menu_list[5].value_max[2]=14;
	menu_list[5].value_max[3]=1;
	menu_list[5].menu_id=5;
	menu_list[5].menu_item_count = 4;
	menu_list[5].menu_pointer=0;
	
	
	strcpy(menu_list[6].menu_name , "Relay Hysteresis");
	strcpy(menu_list[6].menu_strings[0], " Max: %.1f ");
	strcpy(menu_list[6].menu_strings[1], " Min: %.1f ");
	menu_list[6].next_menu_id[0]=1;
	menu_list[6].next_menu_id[1]=1;
	menu_list[6].values[0]=8;
	menu_list[6].values[1]=6;
	menu_list[6].value_resolution[0]=0.10000;
	menu_list[6].value_resolution[1]=0.10000;
	menu_list[6].value_max[0]=14;
	menu_list[6].value_max[1]=14;
	menu_list[6].menu_id=6;
	menu_list[6].menu_item_count = 2;
	menu_list[6].menu_pointer=0;
	
	strcpy(menu_list[7].menu_name , "PID Coefficients");
	strcpy(menu_list[7].menu_strings[0], " P: %d ");
	strcpy(menu_list[7].menu_strings[1], " I: %.1f ");
	strcpy(menu_list[7].menu_strings[2], " D: %.1f ");
	menu_list[7].next_menu_id[0]=1;
	menu_list[7].next_menu_id[1]=1;
	menu_list[7].next_menu_id[2]=1;
	menu_list[7].values[0]=10;
	menu_list[7].values[1]=11;
	menu_list[7].values[2]=12;
	menu_list[7].value_resolution[0]=1;
	menu_list[7].value_resolution[1]=0.1000;
	menu_list[7].value_resolution[2]=0.1000;
	menu_list[7].value_max[0]=100;
	menu_list[7].value_max[1]=100;
	menu_list[7].value_max[2]=100;
	menu_list[7].menu_id=7;
	menu_list[7].menu_item_count = 3;
	menu_list[7].menu_pointer=0;
}
void update_menu_from_variables()
{
	// pid,relay link to coeff and hysteresis
	menu_list[5].values[3]=menu_list[5].values[1];
	if(menu_list[5].values[1] == 1)
	{
		menu_list[5].next_menu_id[3]=6;
	}
	else
	{
		menu_list[5].next_menu_id[3]=7;
	}
	//relay max min restriction
	if(menu_list[6].values[1] > menu_list[6].values[0])
	{
		menu_list[6].values[1] = menu_list[6].values[0];
	}

}
void print_main_page(int active_menu)
{
		char *search_buff;
		int print_offset=0;
		GLCD_ClearScreen();

		for(int i=0 ; i < menu_list[active_menu].menu_item_count ; i++)
		{
			
			if(menu_list[active_menu].font[i] == 1)
			{
						glcd_set_font(Times_New_Roman25x26 ,25,26,32,127);
			}
			if(menu_list[active_menu].font[i] == 0)
			{
						glcd_set_font(Terminal6x8 ,6,8,32,127);
			}
			
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
				for(int j=0 ; j<(int)(menu_list[active_menu].values[i]) ; j++)
				{
					final_menu_strings = strtok(NULL , ",");
				}
			}
			else
			{
				strcpy(final_menu_strings , menu_list[active_menu].menu_strings[i]);
			}

			glcd_draw_string_xy(menu_list[active_menu].x_position[i],menu_list[active_menu].y_position[i],final_menu_strings,0,0,0);
		}
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
				for(int j=0 ; j<(int)(menu_list[active_menu].values[i]) ; j++)
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
				glcd_draw_string_xy(print_offset,i*9+14,final_menu_strings,0,1,0);
			}
			else
			{ 
				glcd_draw_string_xy(print_offset,i*9+14,final_menu_strings,0,0,0);
			}
		}
	
}

void get_user_input(uint8_t *input,int *active_menu)
{
	if(input[0]=='\n')
	{
		if(*active_menu == 0)
		{
			*active_menu = 1;
			GLCD_ClearScreen();
		}
		else
		{
			*active_menu = menu_list[*active_menu].next_menu_id[menu_list[*active_menu].menu_pointer];
			GLCD_ClearScreen();
		}
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
		char sprintf_buff[10];
		if(menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] < menu_list[*active_menu].value_max[menu_list[*active_menu].menu_pointer])
		{
			menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] += menu_list[*active_menu].value_resolution[menu_list[*active_menu].menu_pointer] ;
			sprintf(sprintf_buff,"value:%f\n",menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer]);
			HAL_UART_Transmit(&huart2,sprintf_buff,10,100);
		}
		else if(menu_list[*active_menu].value_resolution[menu_list[*active_menu].menu_pointer] != 0)
		{
			menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] = 0;
			sprintf(sprintf_buff,"value:%f\n",menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer]);
			HAL_UART_Transmit(&huart2,sprintf_buff,10,100);
		}
	}
	if(input[0] == 'a')
	{
		char sprintf_buff[10];
		if(menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] > 0.001)
		{
			menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] -= menu_list[*active_menu].value_resolution[menu_list[*active_menu].menu_pointer] ;
			sprintf(sprintf_buff,"value:%f\n",menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer]);
			HAL_UART_Transmit(&huart2,sprintf_buff,10,100);
		}
		else if(menu_list[*active_menu].value_resolution[menu_list[*active_menu].menu_pointer] != 0)
		{
			menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer] = menu_list[*active_menu].value_max[menu_list[*active_menu].menu_pointer];
			sprintf(sprintf_buff,"value:%f\n",menu_list[*active_menu].values[menu_list[*active_menu].menu_pointer]);
			HAL_UART_Transmit(&huart2,sprintf_buff,10,100);
		}
	}	
}