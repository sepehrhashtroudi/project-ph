//-----------------------------------------------------------------------------
// Copyright:      Nahalco,
// Author:         sepehrhashtroudi
// Remarks:        
// known Problems: none
// Version:        1.1.0
// Description:    max485 send and receive
//								 
//-----------------------------------------------------------------------------

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "usart.h"
#include "gpio.h"
#include "max485.h"

void MAX485_init(int baudrate)
{
	UART_HandleTypeDef huart3;
/* USART2 init function */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = baudrate;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
			Error_Handler();  
	}
}

void MAX485_send_string(char* data,int size,int timeout)
{
	HAL_GPIO_WritePin(RS485_RTS_GPIO_Port,RS485_RTS_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart3,(uint8_t*)data,size,timeout);
	HAL_GPIO_WritePin(RS485_RTS_GPIO_Port,RS485_RTS_Pin,GPIO_PIN_RESET);
}
void MAX485_receive_string(uint8_t* data,int size,int timeout)
{
	HAL_GPIO_WritePin(RS485_RTS_GPIO_Port,RS485_RTS_Pin,GPIO_PIN_RESET);

	HAL_Delay(10);
	HAL_UART_Receive(&huart3,data,size,timeout);
	
}
