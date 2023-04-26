
#ifndef MOTOR_H_
#define MOTOR_H_


#include "main.h"
#include "stdint.h"



typedef struct{
	int16_t velocity;
	int64_t position;
	uint32_t last_counter_value;
} encoder_instance;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim);
void reset_encoder(encoder_instance *encoder_value);



#endif /* MOTOR_H_ */
