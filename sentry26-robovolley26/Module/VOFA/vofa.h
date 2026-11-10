#ifndef STANDARD_ROBOT_C_VOFA_H
#define STANDARD_ROBOT_C_VOFA_H
#include "usart/bsp_uart.h"
#include "typedef.h"
#define VOFA_RX_NUM_MAX 10
#define PARAM_HEADER 0xA5

typedef struct
{
    UART_Instance_t vofa_uart;
    uint16_t tx_num;
    uint16_t rx_num;
    fp32 rxdata[VOFA_RX_NUM_MAX];
    uint32_t rx_cnt;
    uint8_t ready;
} vofa_t;

void vofa_init(UART_HandleTypeDef* huart, uint16_t txnum, uint16_t rxnum);
void vofa_print(const fp32* data);
vofa_t* Get_Vofa_Ptr();

#endif //STANDARD_ROBOT_C_VOFA_H