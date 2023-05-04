#include "lidar_parse.h" 

uint8_t ReceiveQueue[22] = {0};//???
uint8_t ReceiveBuffer[22] = {0};//???
uint8_t OriginalDataTable[22] = {0};//???
uint16_t DataTable[36] = {0};//????
uint8_t ReceiveCnt = 0;
FlagStatus DataCorrectFlag = RESET;
FlagStatus BusyFlag = RESET;
raderDataTypeDef Data[4];

void send_data(void)  
{  
	uint8_t i;
	uint8_t x;
   if (DataCorrectFlag == SET)
   {
	   for (i = 0; i < 22; i++)
	   {
		   OriginalDataTable[i] = ReceiveBuffer[i];
	   }
	   DataCorrectFlag = RESET;
	   for (x = 0; x < 4; x++)
	   {
		   int beg = 4 * x;				
		   Data[x].disvalid = ((0xff & OriginalDataTable[5 + beg]) >> 7) == 0 ? true : false;//Verify data
		   if (Data[x].disvalid == true)
		   {	
			   Data[x].distance = ((0x3f & OriginalDataTable[5 + beg]) << 8) | OriginalDataTable[4 + beg];
			   Data[x].sigintens = (OriginalDataTable[7 + beg] << 8) + OriginalDataTable[6 + beg];
			   Data[x].angle = (OriginalDataTable[1]-0xa0)*4 + x;
			   if (Data[x].angle >= 355)//?????5? Turn clockwise 5 degrees
				   DataTable[(Data[x].angle - 355)/ 10] = Data[x].distance;
			   else
				   DataTable[(Data[x].angle + 5)/ 10] = Data[x].distance;
		   }
	   }
   }
   else
	   return;
}  
