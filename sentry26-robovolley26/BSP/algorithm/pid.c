#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define abs_float(a) (a>0 ? a : -a)

/**
  * @brief pid初始化
  * @param pid: PID结构体指针
  * @param config: PID参数
  * @retval none
  */
void PID_init(pid_t *pid, const pid_config config)
{
    if (pid == NULL)
    {
        return;
    }
    pid->mode = config.mode;
    pid->Kp = config.kp;
    pid->Ki = config.ki;
    pid->Kd = config.kd;
    pid->max_out = config.max_out;
    pid->max_iout = config.max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out[0] =pid->out[1]= 0.0f;
    pid->out_limit_delta[0] = config.out_limit_delta_P;
    pid->out_limit_delta[1] = config.out_limit_delta_N;
	pid->deadzone = config.deadzone;
	pid->flag_multi_Kp = 0;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */

fp32 PID_calc(pid_t *pid, const fp32 ref, const fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->out[1]=pid->out[0];
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
			if(pid->deadzone > 0 && pid->error[0] < pid->deadzone && pid->error[0] > -pid->deadzone)
				pid->out[0] = 0.0f;
			else{
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        if (pid->flag_multi_Kp == 1)//多阶PID开启
        {
          for(uint8_t i = 0; i < pid->multi_Kpid_num; i++){
            if(abs_float(pid->error[0]) <= pid->multi_Kpid_ptr[i][0]){
              pid->Pout =  pid->multi_Kpid_ptr[i][1] * pid->error[0];
              pid->Iout += pid->multi_Kpid_ptr[i][2] * pid->error[0];
              pid->Dout =  pid->multi_Kpid_ptr[i][3] * pid->Dbuf[0];
              break;
            }
          }
        }
        else
        {
          pid->Pout = pid->Kp * pid->error[0];
          pid->Iout += pid->Ki * pid->error[0];
          pid->Dout = pid->Kd * pid->Dbuf[0];
        }
        
        LimitMax(pid->Iout, pid->max_iout);
        pid->out[0] = pid->Pout + pid->Iout + pid->Dout;
				if (pid->Iout*pid->error[0] < 0)
				{
					pid->Iout = 0;
				}
        LimitMax(pid->out[0], pid->max_out);
			}
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out[0] += pid->Pout + pid->Iout + pid->Dout;

        
        LimitMax(pid->out[0], pid->max_out);
    }
    const fp32 temp = pid->out[0] - pid->out[1];
    if(pid->out[1] >= 0.0f)
    {
        if(temp > pid->out_limit_delta[0]){
          pid->out[0] = pid->out[1] + pid->out_limit_delta[0];
        }
        else if (temp < pid->out_limit_delta[1]* -1.0f){
          pid->out[0] = pid->out[1] - pid->out_limit_delta[1];
        }
    }
    else
    {
        if(temp < pid->out_limit_delta[0]*-1.0f){
          pid->out[0] = pid->out[1] - pid->out_limit_delta[0];
        }
        else if (temp > pid->out_limit_delta[1]){
          pid->out[0] = pid->out[1] + pid->out_limit_delta[1];
        }
    }
  
    
    return pid->out[0];
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
void PID_clear(pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out[0] = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

void PID_multi_Kp_init(pid_t *pid,fp32 (* ptr)[4], const uint8_t num){
  pid->flag_multi_Kp = 1;
  pid->multi_Kpid_ptr = ptr;
  pid->multi_Kpid_num = num;
}
