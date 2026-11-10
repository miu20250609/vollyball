#include "vofa.h"
#include "user_lib.h"

void vofacallback(uint8_t* data, uint16_t len);

static vofa_t vofa_ins;

void vofa_init(UART_HandleTypeDef* huart, const uint16_t txnum, const uint16_t rxnum)
{
    vofa_ins.tx_num = txnum;
    if (rxnum > VOFA_RX_NUM_MAX)
        return ;
    vofa_ins.rx_num = rxnum;
    vofa_ins.rx_cnt = 0;
    BSP_UART_Init(&vofa_ins.vofa_uart, huart, 115200, vofacallback, NULL, TX_BUFFER_SIZE, RX_BUFFER_SIZE);
}
//
// void vofacallback(uint8_t* data, uint16_t len)
// {
//     const uint16_t rxlen = sizeof(fp32);
//     if(data[0] != PARAM_HEADER || data[3] != 0 || data[4] != 0x5A)
//         return ;
//     if(data[1] != rxlen)
//         return ;
//     if (data[5] > 0 && data[5] <= vofa_ins.rx_num)
//     {
//         unpack_4bytes_to_floats(&data[7], &vofa_ins.rxdata[data[5]]);
//         vofa_ins.rx_cnt |= (1 << (data[5] - 1));
//     }
//     else if (data[5] == 0xFF)
//     {
//         __HAL_RCC_CLEAR_RESET_FLAGS();
//         osDelay(100);
//         HAL_NVIC_SystemReset();
//     }
//     if (vofa_ins.rx_cnt == (1 << vofa_ins.rx_num) - 1)
//     {
//         vofa_ins.ready = 1;
//         vofa_ins.rx_cnt = 0;
//     }
// }

void vofacallback(uint8_t* data, uint16_t len)
{
    const uint16_t rxlen = 13;
    if (data[0] != 0xA5 || data[3] != 0 || data[4] != 0x5A)
        return ;
    if (data[1] != rxlen)
        return ;
    if (data[5] == 0x01)
    {
      memcpy(vofa_ins.rxdata, data+6, 12);
      vofa_ins.ready = data[18];
    }
}

void vofa_print(const fp32* data)
{
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    uint8_t txdata[vofa_ins.tx_num * sizeof(fp32) + 4];
    memcpy(txdata, data, vofa_ins.tx_num * sizeof(fp32));
    memcpy(txdata + vofa_ins.tx_num * sizeof(fp32), tail, 4);
    BSP_UART_Transmit(&vofa_ins.vofa_uart, (uint8_t*)txdata, vofa_ins.tx_num * sizeof(fp32) + 4, 100);
}

vofa_t* Get_Vofa_Ptr()
{
    return &vofa_ins;
}
