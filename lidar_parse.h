#ifndef __LIDAR_PARSE_H
#define __LIDAR_PARSE_H
/* ????? ----------------------------------------------------------------*/
#include "stm32f4xx.h"

#define bool uint8_t
#define true 1
#define false 0
/* ???? ------------------------------------------------------------------*/
extern uint8_t ReceiveQueue[22];
extern uint8_t ReceiveBuffer[22];
extern uint8_t ReceiveCnt;
extern FlagStatus DataCorrectFlag;
extern uint8_t OriginalDataTable[22];//???
extern uint16_t DataTable[36];//????
extern FlagStatus BusyFlag;
typedef struct
{
	bool disvalid;
	uint16_t distance;
	uint8_t sigintens;
	uint8_t angle;
}DataTypeDef;
extern DataTypeDef Data[4];  

extern send_data();
#endif
