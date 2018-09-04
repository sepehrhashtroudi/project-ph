//-------------------------------------------------------------------------------------------------
// Universal KS0108 driver library
// STM32 MCU low-level driver
// (c) Rados³aw Kwiecieñ, radek@dxp.pl
//-------------------------------------------------------------------------------------------------



#include "stm32f4xx_hal.h"
#include "gpio.h"

#define DISPLAY_STATUS_BUSY	0x80

extern unsigned char screen_x;
extern unsigned char screen_y;

 GPIO_InitTypeDef GPIO_InitStruct;
void Port_in_out_mode(int i)
{
	if(i==1)
	{
	GPIO_InitStruct.Pin = LCD_D2_Pin|LCD_D3_Pin 
                          |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
	GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	if(i==0)
	{
		GPIO_InitStruct.Pin = LCD_D2_Pin|LCD_D3_Pin 
                          |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	
	}
	
}
//-------------------------------------------------------------------------------------------------
// Delay function /for 8MHz/
//-------------------------------------------------------------------------------------------------
void GLCD_Delay(void)
{
  for(int i=0;i<5;i++);
}
//-------------------------------------------------------------------------------------------------
// Enalbe Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_EnableController(unsigned char controller)
{
switch(controller){
	case 0 : HAL_GPIO_WritePin(LCD_CS1_GPIO_Port , LCD_CS1_Pin,GPIO_PIN_SET); break;
	case 1 : HAL_GPIO_WritePin(LCD_CS2_GPIO_Port , LCD_CS2_Pin,GPIO_PIN_SET); break;
	//case 2 : HAL_GPIO_WritePin(LCD_CS1_GPIO_Port , LCD_CS1_Pin,GPIO_PIN_RESET); break;
	}
}
//-------------------------------------------------------------------------------------------------
// Disable Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_DisableController(unsigned char controller)
{
switch(controller){
	case 0 : HAL_GPIO_WritePin(LCD_CS1_GPIO_Port , LCD_CS1_Pin,GPIO_PIN_RESET); break;
	case 1 : HAL_GPIO_WritePin(LCD_CS2_GPIO_Port , LCD_CS2_Pin,GPIO_PIN_RESET); break;
	//case 2 : HAL_GPIO_WritePin(LCD_CS1_GPIO_Port , LCD_CS1_Pin,GPIO_PIN_RESET); break;
	}
}
//-------------------------------------------------------------------------------------------------
// Read Status byte from specified controller (0-2)
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadStatus(unsigned char controller)
{
unsigned char status=0;
Port_in_out_mode(0);
HAL_GPIO_WritePin(LCD_RW_GPIO_Port,LCD_RW_Pin,GPIO_PIN_SET);
HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_RESET);	
GLCD_EnableController(controller);
GLCD_Delay();
HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_SET);
GLCD_Delay();

//status = LL_GPIO_ReadInputPort(GPIOD) & 0XFF;

status += HAL_GPIO_ReadPin(LCD_D0_GPIO_Port,LCD_D0_Pin);
status += 2*HAL_GPIO_ReadPin(LCD_D1_GPIO_Port,LCD_D1_Pin);
status += 4*HAL_GPIO_ReadPin(LCD_D2_GPIO_Port,LCD_D2_Pin);
status += 8*HAL_GPIO_ReadPin(LCD_D3_GPIO_Port,LCD_D3_Pin);
status += 16*HAL_GPIO_ReadPin(LCD_D4_GPIO_Port,LCD_D4_Pin);
status += 32*HAL_GPIO_ReadPin(LCD_D5_GPIO_Port,LCD_D5_Pin);
status += 64*HAL_GPIO_ReadPin(LCD_D6_GPIO_Port,LCD_D6_Pin);
status += 128*HAL_GPIO_ReadPin(LCD_D7_GPIO_Port,LCD_D7_Pin);
HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_RESET);
GLCD_DisableController(controller);

return status;
}
//-------------------------------------------------------------------------------------------------
// Write command to specified controller
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite, unsigned char controller)
{
for(int i=0;i<100000 && GLCD_ReadStatus(controller)&DISPLAY_STATUS_BUSY;i++);
Port_in_out_mode(1);


HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin | LCD_RW_Pin,GPIO_PIN_RESET);
GLCD_Delay();
GLCD_EnableController(controller);
GLCD_Delay();

//LL_GPIO_WriteOutputPort(GPIOD,commandToWrite);
HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,(GPIO_PinState)(commandToWrite & 0x80));
HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,(GPIO_PinState)(commandToWrite & 0x40));
HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,(GPIO_PinState)(commandToWrite & 0x20));
HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,(GPIO_PinState)(commandToWrite & 0x10));
HAL_GPIO_WritePin(LCD_D3_GPIO_Port,LCD_D3_Pin,(GPIO_PinState)(commandToWrite & 0x08));
HAL_GPIO_WritePin(LCD_D2_GPIO_Port,LCD_D2_Pin,(GPIO_PinState)(commandToWrite & 0x04));
HAL_GPIO_WritePin(LCD_D1_GPIO_Port,LCD_D1_Pin,(GPIO_PinState)(commandToWrite & 0x02));
HAL_GPIO_WritePin(LCD_D0_GPIO_Port,LCD_D0_Pin,(GPIO_PinState)(commandToWrite & 0x01));
GLCD_Delay();
HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin ,GPIO_PIN_SET);
GLCD_Delay();
HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin ,GPIO_PIN_RESET);
GLCD_Delay();
GLCD_DisableController(controller);

}

//-------------------------------------------------------------------------------------------------
// Read data from current position
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadData(void)
{
unsigned char tmp=0;
for(int i=0;i<100000 && GLCD_ReadStatus(screen_x / 64)&DISPLAY_STATUS_BUSY;i++);
Port_in_out_mode(0);
HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin | LCD_RW_Pin,GPIO_PIN_SET);
GLCD_EnableController(screen_x / 64);
GLCD_Delay();
HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin ,GPIO_PIN_SET);
GLCD_Delay();
//tmp = LL_GPIO_ReadInputPort(GPIOD) & 0XFF ;
tmp += HAL_GPIO_ReadPin(LCD_D0_GPIO_Port,LCD_D0_Pin);
tmp += 2*HAL_GPIO_ReadPin(LCD_D1_GPIO_Port,LCD_D1_Pin);
tmp += 4*HAL_GPIO_ReadPin(LCD_D2_GPIO_Port,LCD_D2_Pin);
tmp += 8*HAL_GPIO_ReadPin(LCD_D3_GPIO_Port,LCD_D3_Pin);
tmp += 16*HAL_GPIO_ReadPin(LCD_D4_GPIO_Port,LCD_D4_Pin);
tmp += 32*HAL_GPIO_ReadPin(LCD_D5_GPIO_Port,LCD_D5_Pin);
tmp += 64*HAL_GPIO_ReadPin(LCD_D6_GPIO_Port,LCD_D6_Pin);
tmp += 128*HAL_GPIO_ReadPin(LCD_D7_GPIO_Port,LCD_D7_Pin);
HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin ,GPIO_PIN_RESET);
GLCD_DisableController(screen_x / 64);
//screen_x++;
return tmp;
}
//-------------------------------------------------------------------------------------------------
// Write data to current position
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite)
{
for(int i=0;i<100000 && GLCD_ReadStatus(screen_x / 64)&DISPLAY_STATUS_BUSY;i++);
Port_in_out_mode(1);
HAL_GPIO_WritePin(LCD_RW_GPIO_Port,  LCD_RW_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin ,GPIO_PIN_SET);
GLCD_Delay();
//LL_GPIO_WriteOutputPort(GPIOD,dataToWrite);
HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,(GPIO_PinState)(dataToWrite & 0x80));
HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,(GPIO_PinState)(dataToWrite & 0x40));
HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,(GPIO_PinState)(dataToWrite & 0x20));
HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,(GPIO_PinState)(dataToWrite & 0x10));
HAL_GPIO_WritePin(LCD_D3_GPIO_Port,LCD_D3_Pin,(GPIO_PinState)(dataToWrite & 0x08));
HAL_GPIO_WritePin(LCD_D2_GPIO_Port,LCD_D2_Pin,(GPIO_PinState)(dataToWrite & 0x04));
HAL_GPIO_WritePin(LCD_D1_GPIO_Port,LCD_D1_Pin,(GPIO_PinState)(dataToWrite & 0x02));
HAL_GPIO_WritePin(LCD_D0_GPIO_Port,LCD_D0_Pin,(GPIO_PinState)(dataToWrite & 0x01));
GLCD_Delay();
GLCD_EnableController(screen_x / 64);
GLCD_Delay();
HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin ,GPIO_PIN_SET);
GLCD_Delay();
HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin ,GPIO_PIN_RESET);
GLCD_DisableController(screen_x / 64);
screen_x++;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadByteFromROMMemory(char * ptr)
{
  return *ptr;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
