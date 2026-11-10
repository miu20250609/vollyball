#include "nuc.h"
#include "crc.h"
#include "usart.h"
#include "user_lib.h"
#include <string.h>

void NucRxCallback(uint8_t* data, uint16_t len);
void NUC_Chassis_Handler(const uint8_t* data, uint16_t len);
void NUC_Gimbal_Handler(const uint8_t* data, uint16_t len);

static nuc_ctrl_t nuc_instance;

void NUC_Init(UART_HandleTypeDef *nuc_uart)
{
    nuc_ctrl_t* nuc_ptr = &nuc_instance;
    if (nuc_ptr == NULL || nuc_uart == NULL)
        return;
    memset(nuc_ptr, 0, sizeof(nuc_ctrl_t));
    BSP_UART_Init(&nuc_ptr->nuc_uart, nuc_uart, 115200, NucRxCallback, NULL, TX_BUFFER_SIZE, RX_BUFFER_SIZE);
    nuc_ptr->init = 1;
}

void NucRxCallback(uint8_t* data, const uint16_t len)
{
    if (!nuc_instance.init)
        return;
    for (int i = 0; i < len; i++)
    {
        if (data[i] == 0xA5 && i+4+2<len-1)
        {
            uint8_t *rx_data = &data[i];
            if (!Verify_CRC8_Check_Sum(rx_data, 5))
                return ;

            const uint16_t data_length = rx_data[1] | (rx_data[2] << 8);
            if(i+4+2+data_length+2 < len-1)
                if (!Verify_CRC16_Check_Sum(rx_data, data_length + 9))
                    return ;

            const uint16_t cmd_id = (rx_data[6] << 8) | rx_data[5];

            switch (cmd_id)
            {
                case CMD_ID_SELF_DECISION:
                    memcpy(&nuc_instance.self_decision, &rx_data[7], sizeof(uint32_t));
                    break;
                case CMD_ID_CHASSIS_CTRL:
                    NUC_Chassis_Handler(&rx_data[7], data_length);
                    break;
                case CMD_ID_BIG_GIMBAL_CTRL:
                    NUC_Gimbal_Handler(&rx_data[7], data_length);
                    break;
                default :
                    break;
            }

        }
    }
}

void NUC_Chassis_Handler(const uint8_t* data, const uint16_t len)
{
    if (len < 9)
        return;

    unpack_4bytes_to_floats(&data[0], &nuc_instance.vx);
    unpack_4bytes_to_floats(&data[4], &nuc_instance.vy);
    nuc_instance.chassis_spin = data[8] & 0x01;
    nuc_instance.gimbal_scan = (data[8] >> 1) & 0x01;
    nuc_instance.use_cap = (data[8] >> 2) & 0x01;
}

void NUC_Gimbal_Handler(const uint8_t* data, const uint16_t len)
{
    if (len < 4)
        return;

    //unpack_4bytes_to_floats(&data[0], &nuc_instance.gimbal_yaw);原本nuc走can
    
}

void NUC_Referee_Tran(const uint8_t* data, const uint16_t len)
{
    if (!nuc_instance.init || len < 5)
        return;

    BSP_UART_Transmit(&nuc_instance.nuc_uart, data, len, 100);
}

nuc_ctrl_t *Get_NUC_Ctrl_Instance(void)
{
    return &nuc_instance;
}
