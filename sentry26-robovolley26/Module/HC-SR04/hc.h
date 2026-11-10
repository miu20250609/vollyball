#ifndef STANDARD_ROBOT_C_HC_H
#define STANDARD_ROBOT_C_HC_H
#include "typedef.h"
#include "gpio.h"

#define SOUND_SPEED (340.0f) // 340声速
#define HC_DIS_FACTOR (SOUND_SPEED / 10000.0f * 0.5f) // cm

typedef struct
{
    uint64_t us;
    uint64_t echo_time;
    uint64_t last_echo_time;
    fp32 dis, last_dis;
    fp32 vec;
    uint8_t trig;

    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
} hc_t;

void hc_init(GPIO_TypeDef* trig_port, uint16_t trig_pin, GPIO_TypeDef* echo_port, uint16_t echo_pin);
void hc_trig();
void hc_echo();
hc_t* Get_HC_Ptr();

#endif //STANDARD_ROBOT_C_HC_H