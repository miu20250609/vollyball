#ifndef STANDARD_ROBOT_C_LED_H
#define STANDARD_ROBOT_C_LED_H
#include "pwm/bsp_pwm.h"

typedef struct
{
    pwm_instance pwm_red;
    pwm_instance pwm_green;
    pwm_instance pwm_blue;
} led_instance;

void LED_Init(led_instance *led_instance_ptr);
void LED_SetColor(const led_instance *led_instance_ptr, uint32_t aRGB);
void LED_Shine(const led_instance *led_instance_ptr, uint32_t aRGB, uint16_t Hz);

#endif //STANDARD_ROBOT_C_LED_H