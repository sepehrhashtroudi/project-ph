

#define KS0108_SCREEN_WIDTH		128
#define KS0108_SCREEN_HEIGHT	64


#define DISPLAY_SET_Y       0x40
#define DISPLAY_SET_X       0xB8
#define DISPLAY_START_LINE  0xC0
#define DISPLAY_ON_CMD		0x3E
  #define ON	0x01
  #define OFF	0x00
#define DISPLAY_STATUS_BUSY	0x80

void GLCD_Initalize(void);
void GLCD_WriteData(unsigned char);
void GLCD_WriteCommand(unsigned char, unsigned char);
void GLCD_ClearScreen(void);
void GLCD_GoTo(unsigned char, unsigned char);
void GLCD_SetPixel(unsigned char x, unsigned char y, unsigned char color);
void glcd_set_font(const unsigned char  * font_table, unsigned char width, unsigned char height, unsigned char start_char, unsigned char end_char);
void glcd_set_font_with_num(int font_num);
int glcd_draw_char_xy(unsigned char x, unsigned char y, char c,int fast,int invert,int overwrite);
void glcd_draw_string_xy(unsigned char x, unsigned char y, char *c,int fast,int invert, int overwrite);
void GLCD_WriteString(char *);
unsigned int CalcTextWidthEN(char *str);
unsigned char GLCD_ReadStatus(unsigned char controller);
unsigned char GLCD_ReadByteFromROMMemory(char *);
unsigned char GLCD_ReadData(void);
void GLCD_Bitmap(char *, unsigned char, unsigned char, unsigned char, unsigned char);
void GLCD_Clearline(int j);


