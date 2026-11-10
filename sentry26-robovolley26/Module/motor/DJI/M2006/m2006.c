#include "m2006.h"
#include "user_lib.h"

void get_m2006_measure(const uint8_t* rx_data, uint32_t id, void* arg);
M2006_Status_t m2006_init(m2006_instance* m2006_ins, CAN_HandleTypeDef *hcan, const uint32_t txid, const uint8_t num)
{
    if (num > M2006_MAXNUM)
        return M2006_ERROR_INVALID_PARAM;
    if (txid == M2006_TX_1 || txid == M2006_TX_2)
    {
        m2006_ins->can_ins = BSP_CAN_Init(hcan);
        m2006_ins->num = num;
        m2006_ins->txid = txid;

        if (txid == M2006_TX_1)
            for (uint8_t i = 0; i < num; i++)
            {
                BSP_CAN_RegisterStdCallback(m2006_ins->can_ins, M2006_RX_1+i, get_m2006_measure, &m2006_ins->ecd[i]);
            }
        else
            for (uint8_t i = 0; i < num; i++)
            {
                BSP_CAN_RegisterStdCallback(m2006_ins->can_ins, M2006_RX_2+i, get_m2006_measure, &m2006_ins->ecd[i]);
            }

        return M2006_OK;
    }
    return M2006_ERROR_INVALID_PARAM;
}

M2006_Status_t m2006_ctrl(const m2006_instance* m2006_ins, int16_t current[4])
{
    if (m2006_ins == NULL)
        return M2006_ERROR;

    const CAN_Instance_t* can_ins = m2006_ins->can_ins;
    uint8_t txdata[8];
    for (uint8_t i = 0; i < 4; i++)
    {
        int16_constrain(&current[i], -M2006_CURRENT_MAX, M2006_CURRENT_MAX);
        txdata[i * 2] = current[i] >> 8;
        txdata[i * 2 + 1] = current[i];
    }

    if (BSP_CAN_Transmit(can_ins, m2006_ins->txid, CAN_ID_STD, txdata, 8) != CAN_OK)
        return M2006_ERROR;

    return M2006_OK;
}

void get_m2006_measure(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    m2006_ecd_t *ecd = (m2006_ecd_t*)arg;
    ecd->last_ecd = (int16_t)ecd->ecd;
    ecd->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);
    ecd->speed = (int16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    ecd->current = (int16_t)((rx_data)[4] << 8 | (rx_data)[5]);
    ecd->temperature = (rx_data)[6];
}
