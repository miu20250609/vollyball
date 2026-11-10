#include "led.h"
#include "cmsis_os.h"

void LED_Init(led_instance *led_instance_ptr)
{
    BSP_PWM_Init(&led_instance_ptr->pwm_red, &htim5, TIM_CHANNEL_3);
    BSP_PWM_Init(&led_instance_ptr->pwm_green, &htim5, TIM_CHANNEL_2);
    BSP_PWM_Init(&led_instance_ptr->pwm_blue, &htim5, TIM_CHANNEL_1);
}

void LED_SetColor(const led_instance *led_instance_ptr, const uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red, green, blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    BSP_PWM_SetValue(&led_instance_ptr->pwm_red, red);
    BSP_PWM_SetValue(&led_instance_ptr->pwm_green, green);
    BSP_PWM_SetValue(&led_instance_ptr->pwm_blue, blue);
}

void LED_Shine(const led_instance *led_instance_ptr, const uint32_t aRGB, const uint16_t Hz)
{
    for (int i = 0; i < Hz; i++)
    {
        LED_SetColor(led_instance_ptr, aRGB);
        osDelay(1000 / Hz / 2);
        LED_SetColor(led_instance_ptr, 0x00000000);
        osDelay(1000 / Hz / 2);
    }
}