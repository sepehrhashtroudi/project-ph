/**
  ******************************************************************************
  * File Name          : Dsp.c
  * Description        : general dsp functions
  * Author						 : sepehrhashtroudi 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Dsp.h"
#include "defines.h"
 //double filter[51] = {0.0005,0.0006,0.0009,0.0014,0.0020,0.0027,0.0035,0.0046,0.0057,0.0071,0.0085,0.0101,0.0119,0.0137,0.0156,0.0175,0.0194,0.0212,0.0230,0.0246,0.0260,0.0273,0.0283,0.0290,0.0295,0.0296,0.0295,0.0290,0.0283,0.0273,0.0260,0.0246,0.0230,0.0212,0.0194,0.0175,0.0156,0.0137,0.0119,0.0101,0.0085,0.0071,0.0057,0.0046,0.0035,0.0027,0.0020,0.0014,0.0009,0.0006,0.0005};

/**
  * @brief  This function filters data with moving_average
	* @param  sensor_value : input raw sensor data
	* @param  history : history of previous datas
	* @param  sum : sum of the history datas
	* @param  window_pointer : the position of the oldest data in the window
						(which is going to subtracted from sum and overwriten by new data)
  * @retval filtered value
  */
uint16_t best_moving_average(uint16_t sensor_value, uint16_t* history, int32_t* sum,uint16_t* window_pointer)
{
	*sum = *sum + sensor_value;
	*sum = *sum - history[(*window_pointer)];
	history[(*window_pointer)] = sensor_value;
	if((*window_pointer) < filterWindowLength-1)
	{
		(*window_pointer) = (*window_pointer) + 1;
	}
	else
	{
		(*window_pointer) = 0;
	}
	return (*sum / filterWindowLength);
}

/**
  * @brief  This function filters data
	* @param  sensor_value : input raw sensor data
  * @retval None
  */
uint16_t moving_average(uint16_t sensor_value)
{
  static uint32_t history[filterWindowLength];
	float filtered_value = 0;
  
  for(uint16_t i=1 ; i < filterWindowLength ; i++){
    history[i-1] = history[i];
		//filtered_value += history[i-1]*filter[i-1];
  }
  
  history[filterWindowLength - 1] = sensor_value; 
  //filtered_value += sensor_value * filter[filterWindowLength-1];
  //filtered_value /= filterWindowLength;

  return (uint16_t)filtered_value;
}

/**
  * @brief This function spilits dma data buffer into two buffers (evens and odds).
	* @param data_input: pointer to the input data.
	* @param data_output_length: length of splited output data.
	*	@param data_output1: output data buffer1.
	* @param data_output2: output data buffer2.
  */ 
void data_spliter(uint32_t* data_input, uint16_t data_output_length, uint16_t* data_output1, uint16_t* data_output2)
{
	for(int i=0; i<data_output_length;i++)
	{
		data_output1[i] = data_input[2*i];
		data_output2[i] = data_input[2*i + 1];
	}
}

/**
  * @brief This function takes average of data.
	* @param input_data: pointer to the data to be averaged.
	* @param length: length of data array.
  * @retval value of average.
  */ 
uint32_t average(uint16_t* input_data,uint16_t length)
{
	uint32_t average=0;
	for(int i=0 ; i<length ; i++)
	{
	 average+= input_data[i]; 
	}
	average /=length;
	return average;
}
