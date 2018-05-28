//-----------------------------------------------------------------------------
// Copyright:      Nahalco,
// Author:         sepehrhashtroudi
// Remarks:        
// known Problems: none
// Version:        1.1.0
// Description:    max485 send ans receive
//								 
//-----------------------------------------------------------------------------

void MAX485_init(int baudrate);
void MAX485_send_string(char* data,int size,int timeout);
void MAX485_receive_string(uint8_t* data,int size,int timeout);
