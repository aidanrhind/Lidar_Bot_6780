#ifndef MOVING_AVERAGE_H_
#define MOVING_AVERAGE_H_

#include "stdint.h"

#define MOVING_AVERAGE_LENGTH 100
typedef struct{
	int16_t buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	int16_t out;
	int32_t sum;
}mov_aver_instance;


void reset_average_filter(mov_aver_instance* instance);
void apply_average_filter(mov_aver_instance* instance, int16_t input, int16_t *out);





#endif /* MOVING_AVERAGE_H_ */