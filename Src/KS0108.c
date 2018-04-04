//-------------------------------------------------------------------------------------------------
// Universal KS0108 driver library
// (c) Rados³aw Kwiecieñ, radek@dxp.pl
//-------------------------------------------------------------------------------------------------
#include "KS0108.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
//#include "font5x8.h"
  static const char  font5x8[] = {

0x00, 0x00, 0x00, 0x00, 0x00,// (spacja)
0x00, 0x00, 0x5F, 0x00, 0x00,// !
0x00, 0x07, 0x00, 0x07, 0x00,// "
0x14, 0x7F, 0x14, 0x7F, 0x14,// #
0x24, 0x2A, 0x7F, 0x2A, 0x12,// $
0x23, 0x13, 0x08, 0x64, 0x62,// %
0x36, 0x49, 0x55, 0x22, 0x50,// &
0x00, 0x05, 0x03, 0x00, 0x00,// '
0x00, 0x1C, 0x22, 0x41, 0x00,// (
0x00, 0x41, 0x22, 0x1C, 0x00,// )
0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
0x08, 0x08, 0x3E, 0x08, 0x08,// +
0x00, 0x50, 0x30, 0x00, 0x00,// ,
0x08, 0x08, 0x08, 0x08, 0x08,// -
0x00, 0x30, 0x30, 0x00, 0x00,// .
0x20, 0x10, 0x08, 0x04, 0x02,// /
0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
0x00, 0x42, 0x7F, 0x40, 0x00,// 1
0x42, 0x61, 0x51, 0x49, 0x46,// 2
0x21, 0x41, 0x45, 0x4B, 0x31,// 3
0x18, 0x14, 0x12, 0x7F, 0x10,// 4
0x27, 0x45, 0x45, 0x45, 0x39,// 5
0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
0x01, 0x71, 0x09, 0x05, 0x03,// 7
0x36, 0x49, 0x49, 0x49, 0x36,// 8
0x06, 0x49, 0x49, 0x29, 0x1E,// 9
0x00, 0x36, 0x36, 0x00, 0x00,// :
0x00, 0x56, 0x36, 0x00, 0x00,// ;
0x00, 0x08, 0x14, 0x22, 0x41,// <
0x14, 0x14, 0x14, 0x14, 0x14,// =
0x41, 0x22, 0x14, 0x08, 0x00,// >
0x02, 0x01, 0x51, 0x09, 0x06,// ?
0x32, 0x49, 0x79, 0x41, 0x3E,// @
0x7E, 0x11, 0x11, 0x11, 0x7E,// A
0x7F, 0x49, 0x49, 0x49, 0x36,// B
0x3E, 0x41, 0x41, 0x41, 0x22,// C
0x7F, 0x41, 0x41, 0x22, 0x1C,// D
0x7F, 0x49, 0x49, 0x49, 0x41,// E
0x7F, 0x09, 0x09, 0x01, 0x01,// F
0x3E, 0x41, 0x41, 0x51, 0x32,// G
0x7F, 0x08, 0x08, 0x08, 0x7F,// H
0x00, 0x41, 0x7F, 0x41, 0x00,// I
0x20, 0x40, 0x41, 0x3F, 0x01,// J
0x7F, 0x08, 0x14, 0x22, 0x41,// K
0x7F, 0x40, 0x40, 0x40, 0x40,// L
0x7F, 0x02, 0x04, 0x02, 0x7F,// M
0x7F, 0x04, 0x08, 0x10, 0x7F,// N
0x3E, 0x41, 0x41, 0x41, 0x3E,// O
0x7F, 0x09, 0x09, 0x09, 0x06,// P
0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
0x7F, 0x09, 0x19, 0x29, 0x46,// R
0x46, 0x49, 0x49, 0x49, 0x31,// S
0x01, 0x01, 0x7F, 0x01, 0x01,// T
0x3F, 0x40, 0x40, 0x40, 0x3F,// U
0x1F, 0x20, 0x40, 0x20, 0x1F,// V
0x7F, 0x20, 0x18, 0x20, 0x7F,// W
0x63, 0x14, 0x08, 0x14, 0x63,// X
0x03, 0x04, 0x78, 0x04, 0x03,// Y
0x61, 0x51, 0x49, 0x45, 0x43,// Z
0x00, 0x00, 0x7F, 0x41, 0x41,// [
0x02, 0x04, 0x08, 0x10, 0x20,// "\"
0x41, 0x41, 0x7F, 0x00, 0x00,// ]
0x04, 0x02, 0x01, 0x02, 0x04,// ^
0x40, 0x40, 0x40, 0x40, 0x40,// _
0x00, 0x01, 0x02, 0x04, 0x00,// `
0x20, 0x54, 0x54, 0x54, 0x78,// a
0x7F, 0x48, 0x44, 0x44, 0x38,// b
0x38, 0x44, 0x44, 0x44, 0x20,// c
0x38, 0x44, 0x44, 0x48, 0x7F,// d
0x38, 0x54, 0x54, 0x54, 0x18,// e
0x08, 0x7E, 0x09, 0x01, 0x02,// f
0x08, 0x14, 0x54, 0x54, 0x3C,// g
0x7F, 0x08, 0x04, 0x04, 0x78,// h
0x00, 0x44, 0x7D, 0x40, 0x00,// i
0x20, 0x40, 0x44, 0x3D, 0x00,// j
0x00, 0x7F, 0x10, 0x28, 0x44,// k
0x00, 0x41, 0x7F, 0x40, 0x00,// l
0x7C, 0x04, 0x18, 0x04, 0x78,// m
0x7C, 0x08, 0x04, 0x04, 0x78,// n
0x38, 0x44, 0x44, 0x44, 0x38,// o
0x7C, 0x14, 0x14, 0x14, 0x08,// p
0x08, 0x14, 0x14, 0x18, 0x7C,// q
0x7C, 0x08, 0x04, 0x04, 0x08,// r
0x48, 0x54, 0x54, 0x54, 0x20,// s
0x04, 0x3F, 0x44, 0x40, 0x20,// t
0x3C, 0x40, 0x40, 0x20, 0x7C,// u
0x1C, 0x20, 0x40, 0x20, 0x1C,// v
0x3C, 0x40, 0x30, 0x40, 0x3C,// w
0x44, 0x28, 0x10, 0x28, 0x44,// x
0x0C, 0x50, 0x50, 0x50, 0x3C,// y
0x44, 0x64, 0x54, 0x4C, 0x44,// z
0x00, 0x08, 0x36, 0x41, 0x00,// {
0x00, 0x00, 0x7F, 0x00, 0x00,// |
0x00, 0x41, 0x36, 0x08, 0x00,// }
0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
};
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
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_Initalize(void)
{
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
void GLCD_WriteChar(char charToWrite)
{
int i;
charToWrite -= 32; 
for(i = 0; i < 5; i++) 
  GLCD_WriteData(GLCD_ReadByteFromROMMemory((char *)((int)font5x8 + (5 * charToWrite) + i))); 
GLCD_WriteData(0x00);
}

void glcd_set_font(const unsigned char  * font_table, unsigned char width, unsigned char height, unsigned char start_char, unsigned char end_char)
{
	/* Supports variable width fonts */
	FontPointer = font_table;
	FontWidth = width;
	FontHeight = height;
	firstchar = start_char;
	lastchar  = end_char;
}
int glcd_draw_char_xy(unsigned char x, unsigned char y, char c,int fast,int invert)
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
		unsigned char bytes_high;
		unsigned int bytes_per_char;
		unsigned int p;
		
		if ((FontHeight % 8) > 0){
			bytes_high = (FontHeight / 8) + 1;
		}
		else{
			bytes_high = (FontHeight / 8);
		}
		bytes_per_char = FontWidth * bytes_high + 1; /* The +1 is the width byte at the start */
				
		 

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
								GLCD_SetPixel(x+i,y+j*8+bit,0);
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
							else
							{
								//GLCD_SetPixel(x+i,y+j*8+bit,0);
							}
						}
					}
				}									
			}				
		}
		return var_width;	
	
	} 

	void glcd_draw_string_xy(unsigned char x, unsigned char y, char *c,int fast,int invert)
{
	unsigned char  width;

	if (y > (DISPLAY_HEIGHT - FontHeight - 1)) {
		/* Character won't fit */
		return;
	}

	while (*c) {
		width = glcd_draw_char_xy(x,y,*c,fast,invert);
		x += (width );
		
		//x++;
		c++;
	}
	//clear to end of line
	
		
		
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_WriteString(char * stringToWrite)
{
while(*stringToWrite)
  GLCD_WriteChar(*stringToWrite++);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
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





