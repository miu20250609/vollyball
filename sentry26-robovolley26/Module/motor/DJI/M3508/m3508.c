#include "m3508.h"
#include "user_lib.h"

void get_m3508_measure(const uint8_t* rx_data, uint32_t id, void* arg);
M3508_Status_t m3508_init(m3508_instance* m3508_ins, CAN_HandleTypeDef *hcan, const uint32_t txid, const uint8_t num)
{
    if (num > M3508_MAXNUM)
        return M3508_ERROR_INVALID_PARAM;
    if (txid == M3508_TX_1 || txid == M3508_TX_2)
    {
        m3508_ins->can_ins = BSP_CAN_Init(hcan);
        m3508_ins->num = num;
        m3508_ins->txid = txid;

        if (txid == M3508_TX_1)
            for (uint8_t i = 0; i < num; i++)
            {
                BSP_CAN_RegisterStdCallback(m3508_ins->can_ins, M3508_RX_1+i, get_m3508_measure, &m3508_ins->ecd[i]);
            }
        else
            for (uint8_t i = 0; i < num; i++)
            {
                BSP_CAN_RegisterStdCallback(m3508_ins->can_ins, M3508_RX_2+i, get_m3508_measure, &m3508_ins->ecd[i]);
            }

        return M3508_OK;
    }
    return M3508_ERROR_INVALID_PARAM;
}

M3508_Status_t m3508_ctrl(const m3508_instance* m3508_ins, int16_t current[4])
{
    if (m3508_ins == NULL)
        return M3508_ERROR;

    const CAN_Instance_t* can_ins = m3508_ins->can_ins;
    uint8_t txdata[8];
    for (uint8_t i = 0; i < 4; i++)
    {
        int16_constrain(&current[i], -M3508_CURRENT_MAX, M3508_CURRENT_MAX);
        txdata[i * 2] = current[i] >> 8;
        txdata[i * 2 + 1] = current[i];
    }

    if (BSP_CAN_Transmit(can_ins, m3508_ins->txid, CAN_ID_STD, txdata, 8) != CAN_OK)
        return M3508_ERROR;

    return M3508_OK;
}

void get_m3508_measure(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    m3508_ecd_t *ecd = (m3508_ecd_t*)arg;
    ecd->last_ecd = (int16_t)ecd->ecd;
    ecd->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);
    ecd->speed = (int16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    ecd->current = (int16_t)((rx_data)[4] << 8 | (rx_data)[5]);
    ecd->temperature = (rx_data)[6];
}