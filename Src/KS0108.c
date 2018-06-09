//-------------------------------------------------------------------------------------------------
// Universal KS0108 driver library
// (c) Rados³aw Kwiecieñ, radek@dxp.pl
//-------------------------------------------------------------------------------------------------
#include "KS0108.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "font5x8.h"

//-------------------------------------------------------------------------------------------------
extern void GLCD_InitalizePorts(void);
//-------------------------------------------------------------------------------------------------
unsigned char screen_x = 0, screen_y = 0;
const unsigned char 	   *FontPointer; 									// Font	Pointer
unsigned short 						fontSize;											// size of current font
unsigned char 						firstchar;										// first character noumber of current font
unsigned char 						lastchar;											// last character noumber of current font
unsigned char 						charwidth;										// current character width register

unsigned char							FontWidth;										// max width of font
unsigned char							FontHeight;										// max height of font
unsigned char 						FontXScale 			= 1;					// X size of font
unsigned char 						FontYScale 			= 1;					// Y size of font
unsigned char 						FontSpace 			= 1;					// space between char
unsigned char 						bytes_high;
unsigned int 							bytes_per_char;
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_Initalize(void)
{
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	unsigned char i;
	for(i = 0; i < 2; i++)
		GLCD_WriteCommand((DISPLAY_ON_CMD | ON), i);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_GoTo(unsigned char x, unsigned char y)
{
unsigned char i;
screen_x = x;
screen_y = y;

for(i = 0; i < KS0108_SCREEN_WIDTH/64; i++)
  {
  //GLCD_WriteCommand(DISPLAY_SET_Y | 0,i);
  GLCD_WriteCommand(DISPLAY_SET_X | y,i);
  //GLCD_WriteCommand(DISPLAY_START_LINE | 0,i);
  }
GLCD_WriteCommand(DISPLAY_SET_Y | (x % 64), (x / 64));
//GLCD_WriteCommand(DISPLAY_SET_X | y, (x / 64));
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_ClearScreen(void)
{
unsigned char i, j;
for(j = 0; j < KS0108_SCREEN_HEIGHT/8; j++)
  {
  GLCD_GoTo(0,j);
  for(i = 0; i < KS0108_SCREEN_WIDTH; i++)
		{
		//GLCD_GoTo(i,j);
    GLCD_WriteData(0x00);
		}
  }
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
//void GLCD_WriteChar(char charToWrite)
//{
//int i;
//charToWrite -= 32; 
//for(i = 0; i < 5; i++) 
//  //GLCD_WriteData(GLCD_ReadByteFromROMMemory((char *)((int)font5x8 + (5 * charToWrite) + i))); 
//GLCD_WriteData(0x00);
//}
//void GLCD_WriteString(char * stringToWrite)
//{
//while(*stringToWrite)
//  GLCD_WriteChar(*stringToWrite++);
//}

void glcd_set_font(const unsigned char  * font_table, unsigned char width, unsigned char height, unsigned char start_char, unsigned char end_char)
{
	/* Supports variable width fonts */
	FontPointer = font_table;
	FontWidth = width;
	FontHeight = height;
	firstchar = start_char;
	lastchar  = end_char;
	if ((FontHeight % 8) > 0)
	{
		bytes_high = (FontHeight / 8) + 1;
	}
	else
	{
		bytes_high = (FontHeight / 8);
	}
		bytes_per_char = FontWidth * bytes_high + 1; /* The +1 is the width byte at the start */
}

void glcd_set_font_with_num(int font_num)
{
	if(font_num == 0)
	{
		glcd_set_font(Terminal6x8 ,6,8,32,127);
	}
	if(font_num == 1)
	{
		glcd_set_font(Times_New_Roman25x26 ,25,26,32,127);
	}
	
}
// fast write 8 bit data each time and should be used in multiplys of 8 positions
// if fast is set to 0 (slow) you can print in evry position in lcd
// invert print text inverted
// if owerwrite is set to 1 in clears background of text but it is slower
int glcd_draw_char_xy(unsigned char x, unsigned char y, char c,int fast,int invert,int overwrite) 
{
	if (c < firstchar || c > lastchar) 
		{
		c = '.';
		}
	

		/* Font table in MikroElecktronica format
		   - multi row fonts allowed (more than 8 pixels high)
		   - variable width fonts allowed
		   a complete column is written before moving to the next */
		
		unsigned char i;
		unsigned char var_width;
		unsigned int p;


		/* The first byte per character is always the width of the character */

		var_width = FontPointer[ (c - firstchar) * bytes_per_char];
		p=(c - firstchar) * bytes_per_char;
		p++; /* Step over the variable width field */

		/*
		if (x+var_width >= GLCD_LCD_WIDTH || y+font_current.height >= GLCD_LCD_HEIGHT) {
			return;
		}
		*/
		
			for ( i = 0; i < var_width; i++ ) {
			unsigned char j;
			for ( j = 0; j < bytes_high; j++ ) {

				unsigned char	dat = FontPointer[ p + i*bytes_high + j ];
				if(fast == 1)
				{
					if(invert == 1)
					{
						GLCD_GoTo(x+i,y+j);
						GLCD_WriteData(~dat);
					}
					else 
					{
						GLCD_GoTo(x+i,y+j);
						GLCD_WriteData(dat);
					}
				}
				else
				{
					unsigned char bit;
					for (bit = 0; bit < 8; bit++) 
					{						
						if (x+i >= DISPLAY_WIDTH || y+j*8+bit >= DISPLAY_HEIGHT) 
						{
							/* Don't write past the dimensions of the LCD, skip the entire char */
							return 0;
						}	
						/* We should not write if the y bit exceeds font height */
						if ((j*8 + bit) >= FontHeight) 
						{
							/* Skip the bit */
							continue;
						}
						if(invert == 1)
						{
							if (dat & (1<<bit))
							{
								if(overwrite == 1)
								{
									GLCD_SetPixel(x+i,y+j*8+bit,0);
								}
							}
							else 
							{
								GLCD_SetPixel(x+i,y+j*8+bit,1);
							}
						}
						else 
						{
							if (dat & (1<<bit))
							{
								GLCD_SetPixel(x+i,y+j*8+bit,1);
							}
							else if(overwrite == 1)
							{
								GLCD_SetPixel(x+i,y+j*8+bit,0);
							}
						}
					}
				}									
			}				
		}
		return var_width;	
	
	} 

	void glcd_draw_string_xy(unsigned char x, unsigned char y, char *c,int fast,int invert,int overwrite)
{
	unsigned char  width;

	if (y > (DISPLAY_HEIGHT - FontHeight - 1)) {
		/* Character won't fit */
		return;
	}

	while (*c) {
		width = glcd_draw_char_xy(x,y,*c,fast,invert,overwrite);
		x += (width );
		
		//x++;
		c++;
	}
	//clear to end of line
//	for(int j=x;j<DISPLAY_WIDTH-3;j++)
//		{
//			for(int i=y;i<y+FontHeight;i++)
//			{
//				GLCD_SetPixel(j, i, 0);
//			}
//		}
}

unsigned int CalcTextWidthEN(char *str)
{
	unsigned int 		strSize = 0;
	unsigned char 	c;
	unsigned int 		i = 0;	

	while(str[i])
	{
		c = str[i++];
		
		if(c == '\n')			continue;
		

		  if((c < firstchar) || (c > lastchar)) 
				charwidth = FontWidth;
			else
			{
				charwidth = FontPointer[(c - firstchar) * bytes_per_char];

				//english spesial fonts!
				if (((c >= 0xd4) && (c <= 0xda)) || 
		    		((c >= 0xe7) && (c <= 0xec)) ||
				  	 (c == 0xd1))

					charwidth = 0;
			}
			
			strSize += charwidth;
		
		
		
	}//while
	
	return(strSize);
}	//*CalcTextWidthEN

void GLCD_SetPixel(unsigned char x, unsigned char y, unsigned char color)
{

unsigned char tmp;

GLCD_GoTo(x, (y / 8));
tmp = GLCD_ReadData();
tmp = GLCD_ReadData();
GLCD_GoTo(x, (y / 8));

	if(color == 0)
	{
		tmp &= ~(1 << (y % 8));
	}
	else
	{
		tmp |= (1 << (y % 8));
	}
GLCD_WriteData(tmp);


}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_Bitmap(char * bmp, unsigned char x, unsigned char y, unsigned char dx, unsigned char dy)
{
unsigned char i, j;
for(j = 0; j < dy / 8; j++)
  {
  GLCD_GoTo(x,y + j);
  for(i = 0; i < dx; i++) 
    GLCD_WriteData(GLCD_ReadByteFromROMMemory(bmp++));
  }
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------





