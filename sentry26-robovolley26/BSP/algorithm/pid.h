#ifndef PID_H
#define PID_H
#include "typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;

    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;
    fp32 max_iout;

    fp32 set;
    fp32 fdb;

    fp32 out[2];
    fp32 out_limit_delta[2];
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];
    fp32 error[3];
	
    fp32 deadzone;

    fp32 flag_multi_Kp;
    uint8_t multi_Kpid_num;
    fp32 (* multi_Kpid_ptr)[4];

} pid_t;

typedef struct
{
    uint8_t mode;
    fp32 kp, ki, kd;
    fp32 max_out, max_iout;
    fp32 out_limit_delta_P, out_limit_delta_N;
    fp32 deadzone;
} pid_config;

void PID_init(pid_t *pid, pid_config config);
fp32 PID_calc(pid_t *pid, fp32 ref, fp32 set);
void PID_clear(pid_t *pid);
void PID_multi_Kp_init(pid_t *pid,fp32  (*ptr)[4], uint8_t num);

#endif
