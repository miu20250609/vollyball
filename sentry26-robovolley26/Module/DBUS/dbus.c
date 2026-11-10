#include "dbus.h"
#include "usart.h"
#include "dwt/bsp_dwt.h"

static rc_instance* rc_ins_ptr = NULL;

void RemoteDataProcess_UART(uint8_t *data, uint16_t len);
void RemoteDataProcess_CAN(const uint8_t *data, uint32_t id, void* arg);
void dbus_init(rc_instance* rc_ins, const RC_MODE mode, CAN_HandleTypeDef* hcan)
{
    memset(&rc_ins->rc_data, 0, sizeof(rc_data_t));
    if (mode == RC_DIRECT)
        rc_ins->rc_data.mode = RC_DIRECT;
    else if (mode == RC_CAN)
        rc_ins->rc_data.mode = RC_CAN;
    if (rc_ins->rc_data.mode == RC_DIRECT)
        BSP_UART_Init(&rc_ins->dbus_usart, &DBUS_HUART, 100000, RemoteDataProcess_UART, NULL, 0, DBUS_MAX_LEN);
    else
    {
        rc_ins->dbus_can = BSP_CAN_Init(hcan);
        BSP_CAN_RegisterStdCallback(rc_ins->dbus_can, DBUS_RXID, RemoteDataProcess_CAN, &rc_ins->rc_data);
    }

    rc_ins_ptr = rc_ins;
}

void RemoteDataProcess_UART(uint8_t *data, const uint16_t len)
{
    rc_data_t* rc_data = &rc_ins_ptr->rc_data;
    if (data == NULL || len != 18)
    {
        return;
    }
    rc_data->ch0 = (int16_t)(((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF);
    rc_data->ch1 = (int16_t)((((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF);
    rc_data->ch2 = (int16_t)((((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) | ((int16_t)data[4] << 10)) & 0x07FF);
    rc_data->ch3 = (int16_t)((((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF);
    rc_data->roll = (int16_t)(data[16] | (data[17] << 8));
    rc_data->ch0 -= RC_CH_VALUE_OFFSET;
    rc_data->ch1 -= RC_CH_VALUE_OFFSET;
    rc_data->ch2 -= RC_CH_VALUE_OFFSET;
    rc_data->ch3 -= RC_CH_VALUE_OFFSET;
    rc_data->roll -= RC_CH_VALUE_OFFSET;
    rc_data->s1 = ((data[5] >> 4) & 0x0003);
    rc_data->s2 = ((data[5] >> 4) & 0x000C) >> 2;
    rc_ins_ptr->last_online = DWT_GetTimeline_us();
}

void RemoteDataProcess_CAN(const uint8_t *data, uint32_t id, void* arg)
{
    if (data == NULL || arg == NULL)
    {
        return;
    }
    rc_data_t* rc_data = (rc_data_t*)arg;

    rc_data->ch0 = (int16_t)(((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF);
    rc_data->ch1 = (int16_t)((((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF);
    rc_data->ch2 = (int16_t)((((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) | ((int16_t)data[4] << 10)) & 0x07FF);
    rc_data->ch3 = (int16_t)((((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF);
    rc_data->roll = (int16_t)(data[6] | (data[7] << 8));
    rc_data->ch0 -= RC_CH_VALUE_OFFSET;
    rc_data->ch1 -= RC_CH_VALUE_OFFSET;
    rc_data->ch2 -= RC_CH_VALUE_OFFSET;
    rc_data->ch3 -= RC_CH_VALUE_OFFSET;
    rc_data->roll -= RC_CH_VALUE_OFFSET;
    rc_data->s1 = ((data[5] >> 4) & 0x0003);
    rc_data->s2 = ((data[5] >> 4) & 0x000C) >> 2;
    rc_ins_ptr->last_online = DWT_GetTimeline_us();
}

