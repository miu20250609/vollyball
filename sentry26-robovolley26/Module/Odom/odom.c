#include "odom.h"
#include "crc.h"
#include "user_lib.h"
#include "usart/bsp_uart.h"

void odom_rxcallback(uint8_t* data, uint16_t len);
static odom_t* odom_ins = NULL;

void Odom_Init(odom_t* odom_ptr, UART_HandleTypeDef* huart)
{
    //memset(odom_ptr, 0, sizeof(odom_t));
    BSP_UART_Init(&odom_ptr->odom_uart, huart, 115200, odom_rxcallback, NULL, TX_BUFFER_SIZE, RX_BUFFER_SIZE);
    odom_ins = odom_ptr;
}

void odom_pos_handler(const uint8_t* data, uint16_t len);
void odom_rxcallback(uint8_t* data, const uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        if (data[0] != ODOM_HEADER_1 || data[1] != ODOM_HEADER_2) return ;
        const uint8_t data_len = data[2];
        if (len < ODOM_HEADER_LEN + 1 + data_len + ODOM_CRC16_LEN) return ;
        if (!Verify_CRC8_Check_Sum(data, ODOM_HEADER_LEN)) return ;
        if (!Verify_CRC16_Check_Sum(data, ODOM_HEADER_LEN + 1 + data_len + ODOM_CRC16_LEN)) return ;

        const uint8_t cmd_id = data[ODOM_HEADER_LEN]; // 实际只有一个命令ID
        if (cmd_id == 0x01)
            odom_pos_handler(&data[ODOM_HEADER_LEN + 1], data_len);
    }
}

void odom_pos_handler(const uint8_t* data, const uint16_t len)
{
    if (len != 20) return ;
    memcpy(&odom_ins->x, &data[0], sizeof(fp32));
    memcpy(&odom_ins->y, &data[4], sizeof(fp32));
    memcpy(&odom_ins->theta, &data[8], sizeof(fp32));
    memcpy(&odom_ins->vx, &data[12], sizeof(fp32));
    memcpy(&odom_ins->vy, &data[16], sizeof(fp32));
}

void OdomCmd_PackPacket(const uint8_t *data_in, const uint16_t data_len, const uint16_t cmd_id)
{
    const uint16_t total_len = ODOM_HEADER_LEN + 1 + data_len + ODOM_CRC16_LEN;
    uint8_t data_out[total_len];
    uint32_t offset = 0;
    data_out[offset++] = ODOM_HEADER_1;
    data_out[offset++] = ODOM_HEADER_2;
    data_out[offset++] = data_len & 0xFF;

    data_out[offset++] = 0;
    data_out[offset++] = 0;
    data_out[offset++] = cmd_id & 0xFF;
    if (data_in != NULL && data_len > 0) {
        memcpy(&data_out[offset], data_in, data_len);
        offset += data_len;
    }
    Append_CRC8_Check_Sum(data_out, ODOM_HEADER_LEN);
    Append_CRC16_Check_Sum(data_out, total_len);

    BSP_UART_Transmit(&odom_ins->odom_uart, data_out, sizeof(data_out), 100);
}

void odom_send_speed(const int16_t speed[3])
{
    static uint8_t send[6] = {0};
    memcpy(send, speed, 6);
    OdomCmd_PackPacket(send, sizeof(send), 0x02);
}