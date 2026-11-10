#ifndef STANDARD_ROBOT_C_ODOM_H
#define STANDARD_ROBOT_C_ODOM_H
#include "typedef.h"
#include "usart/bsp_uart.h"

#define ODOM_HEADER_1 0xA5
#define ODOM_HEADER_2 0x5A
#define ODOM_HEADER_LEN 5
#define ODOM_CRC16_LEN 2

typedef struct
{
    UART_Instance_t odom_uart;
    fp32 x;      // 机器人在全局坐标系中的x坐标，单位米
    fp32 y;      // 机器人在全局坐标系中的y坐标，单位米
    fp32 theta;  // 机器人朝向，单位弧度，0表示朝向x轴正方向，逆时针为正
    fp32 vx, vy;
} odom_t;

void Odom_Init(odom_t* odom_ptr, UART_HandleTypeDef* huart);
void odom_send_speed(const int16_t speed[3]);

#endif //STANDARD_ROBOT_C_ODOM_H