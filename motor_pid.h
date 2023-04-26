
#ifndef INC_MOTOR_PID_H_
#define INC_MOTOR_PID_H_

#include "motor.h"
#include "stdint.h"
typedef struct{
uint8_t kp;
uint8_t ki;
uint8_t kd;
int16_t error_prev;
int32_t error_integral;
int16_t output;
}PID_instance;

void set_PID(PID_instance *pid_instance, uint8_t kp, uint8_t ki, uint8_t);
void apply_PID(PID_instance *pid_instance, int16_t input_error, uint16_t sampling_rate);
void reset_PID(PID_instance *pid_instance);

#endif /* INC_MOTOR_PID_H_ */