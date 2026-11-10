#include "bsp_pwm.h"

void BSP_PWM_Init(pwm_instance *pwm_instance_ptr, TIM_HandleTypeDef *tim_pwmHandle, const uint32_t Channel)
{
    if (pwm_instance_ptr == NULL || tim_pwmHandle == NULL)
        return;

    pwm_instance_ptr->tim_pwmHandle = tim_pwmHandle;
    pwm_instance_ptr->Channel = Channel;
    HAL_TIM_Base_Start(pwm_instance_ptr->tim_pwmHandle);
    HAL_TIM_PWM_Start(pwm_instance_ptr->tim_pwmHandle, pwm_instance_ptr->Channel);
}

void BSP_PWM_SetValue(const pwm_instance *pwm_instance_ptr, uint16_t value)
{
    if (value > pwm_instance_ptr->tim_pwmHandle->Instance->ARR)
        value = pwm_instance_ptr->tim_pwmHandle->Instance->ARR;

    switch (pwm_instance_ptr->Channel)
    {
    case TIM_CHANNEL_1:
        pwm_instance_ptr->tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        pwm_instance_ptr->tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        pwm_instance_ptr->tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        pwm_instance_ptr->tim_pwmHandle->Instance->CCR4 = value;
        break;
    default:
        break;
    }
}
