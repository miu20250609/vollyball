#include "nuc.h"
#include "crc.h"
#include "usart.h"
#include "user_lib.h"
#include <string.h>

#define NUC_SOF 0xA5U
#define NUC_HEADER_SIZE 5U
#define NUC_PAYLOAD_OFFSET 7U
#define NUC_FRAME_OVERHEAD (NUC_PAYLOAD_OFFSET + 2U)

void NucRxCallback(uint8_t* data, uint16_t len);
void NUC_Chassis_Handler(const uint8_t* data, uint16_t len);
void NUC_Gimbal_Handler(const uint8_t* data, uint16_t len);

static nuc_ctrl_t nuc_instance;
static uint8_t nuc_rx_buffer[RX_BUFFER_SIZE];
static uint16_t nuc_rx_size;

void NUC_Init(UART_HandleTypeDef *nuc_uart)
{
    nuc_ctrl_t* nuc_ptr = &nuc_instance;
    if (nuc_ptr == NULL || nuc_uart == NULL)
        return;
    memset(nuc_ptr, 0, sizeof(nuc_ctrl_t));
    nuc_rx_size = 0;
    BSP_UART_Init(&nuc_ptr->nuc_uart, nuc_uart, 115200, NucRxCallback, NULL, TX_BUFFER_SIZE, RX_BUFFER_SIZE);
    nuc_ptr->init = 1;
}

static void NUC_DispatchFrame(const uint8_t* frame, const uint16_t payload_size)
{
    const uint16_t cmd_id = (frame[6] << 8) | frame[5];
    const uint8_t *payload = &frame[NUC_PAYLOAD_OFFSET];
    switch (cmd_id)
    {
        case CMD_ID_SELF_DECISION:
            if (payload_size >= sizeof(nuc_instance.self_decision))
                memcpy(&nuc_instance.self_decision, payload, sizeof(nuc_instance.self_decision));
            break;
        case CMD_ID_CHASSIS_CTRL:
            NUC_Chassis_Handler(payload, payload_size);
            break;
        case CMD_ID_BIG_GIMBAL_CTRL:
            NUC_Gimbal_Handler(payload, payload_size);
            break;
        default:
            break;
    }
}

static void NUC_ParseRxBuffer(void)
{
    uint16_t consumed = 0;
    while (consumed < nuc_rx_size)
    {
        uint8_t *frame = &nuc_rx_buffer[consumed];
        const uint16_t available = nuc_rx_size - consumed;
        if (frame[0] != NUC_SOF)
        {
            consumed++;
            continue;
        }
        if (available < NUC_HEADER_SIZE)
            break;

        const uint16_t payload_size = frame[1] | (frame[2] << 8);
        if (!Verify_CRC8_Check_Sum(frame, NUC_HEADER_SIZE) ||
            payload_size > sizeof(nuc_rx_buffer) - NUC_FRAME_OVERHEAD)
        {
            consumed++;
            continue;
        }

        const uint16_t frame_size = payload_size + NUC_FRAME_OVERHEAD;
        if (available < frame_size)
            break;
        if (!Verify_CRC16_Check_Sum(frame, frame_size))
        {
            consumed++;
            continue;
        }

        NUC_DispatchFrame(frame, payload_size);
        consumed += frame_size;
    }

    if (consumed != 0)
    {
        nuc_rx_size -= consumed;
        memmove(nuc_rx_buffer, &nuc_rx_buffer[consumed], nuc_rx_size);
    }
}

void NucRxCallback(uint8_t* data, const uint16_t len)
{
    if (!nuc_instance.init)
        return;

    // DMA 空闲回调可能带来半帧或多帧，保留还没收齐的候选帧。
    for (uint16_t offset = 0; offset < len;)
    {
        uint16_t copy_size = sizeof(nuc_rx_buffer) - nuc_rx_size;
        if (copy_size > len - offset)
            copy_size = len - offset;
        memcpy(&nuc_rx_buffer[nuc_rx_size], &data[offset], copy_size);
        nuc_rx_size += copy_size;
        offset += copy_size;
        NUC_ParseRxBuffer();
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
