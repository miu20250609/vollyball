#ifndef STANDARD_ROBOT_C_BSP_PWM_H
#define STANDARD_ROBOT_C_BSP_PWM_H

#include "typedef.h"
#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *tim_pwmHandle; // PWM定时器句柄
    uint8_t Channel; // PWM通道
} pwm_instance;

void BSP_PWM_Init(pwm_instance *pwm_instance_ptr, TIM_HandleTypeDef *tim_pwmHandle, uint32_t Channel);
void BSP_PWM_SetValue(const pwm_instance *pwm_instance_ptr, uint16_t value);

#endif
