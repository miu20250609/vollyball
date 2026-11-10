#ifndef STANDARD_ROBOT_C_NUC_H
#define STANDARD_ROBOT_C_NUC_H
#include "usart/bsp_uart.h"
#include "typedef.h"

/**
 * @brief NUC（雷达小电脑）与底盘C板之间的UART通信协议
 *        帧格式: 0xA5 | data_length(2B) | seq | CRC8 | cmd_id(2B) | data | CRC16
 */
typedef enum
{
    CMD_ID_SELF_DECISION = 0x401,   // 自主决策指令（本工程暂未使用）
    CMD_ID_CHASSIS_CTRL = 0x402,    // 底盘控制指令：vx, vy + 标志位
    CMD_ID_BIG_GIMBAL_CTRL = 0x403, // 大云台控制指令：gimbal_yaw（本工程暂无云台）
} nuc_cmd_id_t;

typedef struct
{
    UART_Instance_t nuc_uart;
    fp32 vx, vy, vw;
    fp32 gimbal_yaw;
    uint8_t gimbal_scan, last_gimbal_scan;
    uint8_t chassis_spin;
    uint8_t use_cap;
    uint32_t self_decision;
    uint8_t init;
} nuc_ctrl_t;

void NUC_Init(UART_HandleTypeDef *nuc_uart);
nuc_ctrl_t *Get_NUC_Ctrl_Instance(void);
void NUC_Referee_Tran(const uint8_t* data, uint16_t len);

#endif //STANDARD_ROBOT_C_NUC_H
