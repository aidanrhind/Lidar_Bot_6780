# include "motor_pid.h"

#define KI_MAX 2000000
#define PID_MAX 8000

void set_PID(PID_instance * pid_instance, uint8_t kp, uint8_t ki, uint8_t kd)
{
	pid_instance->kp = kp;
	pid_instance->ki = ki;
	pid_instance->kd = kd;
	
}
void reset_pid(PID_instance *pid_instance)
{
	pid_instance->kp =  0;
	pid_instance ->ki = 0;
	pid_instance->kd =  0;
	pid_instance ->error_integral = 0;
}
void apply_PID(PID_instance *pid_instance, int16_t input_error, uint16_t sampling_rate)
{
	pid_instance ->error_integral += input_error;
	if(pid_instance->error_integral > KI_MAX)
	{
		pid_instance->error_integral = KI_MAX;
	}
	if(pid_instance->error_integral < -KI_MAX)
	{
		pid_instance->error_integral = -KI_MAX;
	}
	pid_instance ->output = pid_instance->kp * input_error + pid_instance->ki * (pid_instance->error_integral) / sampling_rate + pid_instance->kd * sampling_rate * (input_error - pid_instance->error_prev);

	if(pid_instance->output >= PID_MAX)
	{
		pid_instance->output = PID_MAX;
	}
	if(pid_instance->output <= -PID_MAX)
	{
		pid_instance->output = -PID_MAX;
	}
	pid_instance->error_prev = input_error;
}

