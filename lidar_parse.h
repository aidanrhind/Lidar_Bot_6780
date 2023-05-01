#ifndef __LIDAR_PARSE_H
#define __LIDAR_PARSE_H
/* ????? ----------------------------------------------------------------*/
#include "stm32f4xx.h"

#define bool uint8_t
#define true 1
#define false 0
/* ???? ------------------------------------------------------------------*/
extern uint8_t raderReceiveQueue[22];
extern uint8_t raderReceiveBuffer[22];
extern uint8_t raderReceiveCnt;
extern FlagStatus raderDataCorrectFlag;
extern uint8_t raderOriginalDataTable[22];//???
extern uint16_t raderDataTable[36];//????
extern FlagStatus raderBusyFlag;
typedef struct
{
	bool disvalid;
	uint16_t distance;
	uint8_t sigintens;
	uint8_t angle;
}raderDataTypeDef;
extern raderDataTypeDef raderData[4];  
/* ???? ------------------------------------------------------------------*/
extern void start_parse_XV11();
#endif