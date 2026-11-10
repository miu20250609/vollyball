#include "hc.h"
#include "dwt/bsp_dwt.h"

static hc_t hc = {0};

void hc_init(GPIO_TypeDef* trig_port, const uint16_t trig_pin, GPIO_TypeDef* echo_port, const uint16_t echo_pin)
{
    hc.trig_port = trig_port;
    hc.trig_pin = trig_pin;
    hc.echo_port = echo_port;
    hc.echo_pin = echo_pin;
    hc.dis = 0;
    hc.echo_time = 0, hc.us = 0;
    hc.last_dis = 0, hc.last_echo_time = 0;
    hc.trig = 0;
}

void hc_trig()
{
    if (hc.trig) return;

    HAL_GPIO_WritePin(hc.trig_port, hc.trig_pin, GPIO_PIN_SET);
    DWT_Delay_us(15);
    HAL_GPIO_WritePin(hc.trig_port, hc.trig_pin, GPIO_PIN_RESET);
    hc.trig = 1;
    hc.echo_time = 0;
}

void hc_echo()
{
    if (!hc.trig) return;

    if (HAL_GPIO_ReadPin(hc.echo_port, hc.echo_pin) == GPIO_PIN_SET && !hc.echo_time)
    {
        hc.echo_time = DWT_GetTimeline_us();
    }
    else if (HAL_GPIO_ReadPin(hc.echo_port, hc.echo_pin) == GPIO_PIN_RESET && hc.echo_time > 0)
    {
        hc.us = DWT_GetTimeline_us() - hc.echo_time;
        hc.dis = (fp32)hc.us * HC_DIS_FACTOR;
        //if (hc.dis > 1000.0f) hc.dis = hc.last_dis;
        hc.vec = (hc.dis - hc.last_dis) * 0.01f / ((fp32)(hc.echo_time - hc.last_echo_time) / 1000000.0f);
        hc.last_echo_time = hc.echo_time;
        hc.last_dis = hc.dis;
        hc.echo_time = 0;
        hc.trig = 0;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == hc.echo_pin)
    {
        hc_echo();
    }

}

hc_t* Get_HC_Ptr()
{
    return &hc;
}