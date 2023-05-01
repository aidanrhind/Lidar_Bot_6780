#include "lidar_parse.h" 

/* ???? ------------------------------------------------------------------*/
uint8_t raderReceiveQueue[22] = {0};//???
uint8_t raderReceiveBuffer[22] = {0};//???
uint8_t raderOriginalDataTable[22] = {0};//???
uint16_t raderDataTable[36] = {0};//????
uint8_t raderReceiveCnt = 0;
FlagStatus raderDataCorrectFlag = RESET;
FlagStatus raderBusyFlag = RESET;
raderDataTypeDef raderData[4];

/* ???? ------------------------------------------------------------------*/
  
/************************************************************  
 * Function: start_parse_XV11  
 * Description: Began to parse lidar data,it is recommended that the function be placed in a loop
 * Input parameter: void  
 * Return parameter: void  
 * Calls:  
 * Called by:  
* Author: zilin Wang     Build_Date:2017-5-24 17:24:38
 * Other:  
*************************************************************/  
   
void start_parse_XV11(void)  
{  
	uint8_t i;
	uint8_t x;
   if (raderDataCorrectFlag == SET)
   {
	   for (i = 0; i < 22; i++)
	   {
		   raderOriginalDataTable[i] = raderReceiveBuffer[i];
	   }
	   raderDataCorrectFlag = RESET;
	   for (x = 0; x < 4; x++)
	   {
		   int beg = 4 * x;				
		   raderData[x].disvalid = ((0xff & raderOriginalDataTable[5 + beg]) >> 7) == 0 ? true : false;//Verify data
		   if (raderData[x].disvalid == true)
		   {	
			   raderData[x].distance = ((0x3f & raderOriginalDataTable[5 + beg]) << 8) | raderOriginalDataTable[4 + beg];
			   raderData[x].sigintens = (raderOriginalDataTable[7 + beg] << 8) + raderOriginalDataTable[6 + beg];
			   raderData[x].angle = (raderOriginalDataTable[1]-0xa0)*4 + x;
			   if (raderData[x].angle >= 355)//?????5? Turn clockwise 5 degrees
				   raderDataTable[(raderData[x].angle - 355)/ 10] = raderData[x].distance;
			   else
				   raderDataTable[(raderData[x].angle + 5)/ 10] = raderData[x].distance;
		   }
	   }
   }
   else
	   return;
}  