#include "nuc.h"
#include "crc.h"
#include "usart.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>
#include "can/bsp_can.h"
#include "stdbool.h"

#define NUC_SOF 0xA5U
#define NUC_HEADER_SIZE 5U
#define NUC_PAYLOAD_OFFSET 7U
#define NUC_FRAME_OVERHEAD (NUC_PAYLOAD_OFFSET + 2U)
#define NUC_CHASSIS_TIMEOUT_MS 200U

static void NucRxCallback(uint8_t* data, uint16_t len);
static void NUC_CanRxCallback(const uint8_t* data, uint32_t id, void* arg);
static void NUC_Chassis_Handler(const uint8_t* data, uint16_t len);
static void NUC_Gimbal_Handler(const uint8_t* data, uint16_t len);
static void NUC_UpdateChassis(fp32 vx, fp32 vy, uint8_t flags);

static nuc_ctrl_t nuc_instance;
static uint8_t nuc_rx_buffer[RX_BUFFER_SIZE];
static uint16_t nuc_rx_size;
static uint32_t nuc_chassis_rx_tick;
static bool nuc_chassis_received;

void NUC_Init(UART_HandleTypeDef *nuc_uart)
{
    if (nuc_uart == NULL || nuc_instance.init)
        return;
    if (BSP_UART_Init(&nuc_instance.nuc_uart, nuc_uart, 115200, NucRxCallback, NULL,
                      TX_BUFFER_SIZE, RX_BUFFER_SIZE) == UART_OK)
        nuc_instance.init = 1;
}

CAN_Status_t NUC_Init_CAN(CAN_HandleTypeDef *nuc_can)
{
    if (nuc_instance.init)
        return CAN_ERROR;
    CAN_Instance_t* can = BSP_CAN_Init(nuc_can);
    if (can == NULL)
        return CAN_ERROR;

    CAN_Status_t status = BSP_CAN_RegisterStdDataCallback(can, CMD_ID_SELF_DECISION, 4, NUC_CanRxCallback, NULL);
    if (status == CAN_OK)
        status = BSP_CAN_RegisterStdDataCallback(can, CMD_ID_CHASSIS_CTRL, 5, NUC_CanRxCallback, NULL);
    if (status == CAN_OK)
        status = BSP_CAN_RegisterStdDataCallback(can, CMD_ID_BIG_GIMBAL_CTRL, 2, NUC_CanRxCallback, NULL);
    if (status == CAN_OK)
        nuc_instance.init = 1;
    return status;
}

static int16_t NUC_ReadInt16LE(const uint8_t* data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void NUC_CanRxCallback(const uint8_t* data, uint32_t id, void* arg)
{
    (void)arg;
    if (!nuc_instance.init)
        return;

    switch (id)
    {
        case CMD_ID_SELF_DECISION:
            nuc_instance.self_decision = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                                         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
            break;
        case CMD_ID_CHASSIS_CTRL:
            NUC_UpdateChassis(NUC_ReadInt16LE(&data[0]) / 1000.0f,
                              NUC_ReadInt16LE(&data[2]) / 1000.0f, data[4]);
            break;
        case CMD_ID_BIG_GIMBAL_CTRL:
            nuc_instance.gimbal_yaw = NUC_ReadInt16LE(data) / 100.0f;
            break;
        default:
            break;
    }
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

static void NucRxCallback(uint8_t* data, const uint16_t len)
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

static void NUC_Chassis_Handler(const uint8_t* data, const uint16_t len)
{
    if (len < 9)
        return;

    fp32 vx, vy;
    unpack_4bytes_to_floats(&data[0], &vx);
    unpack_4bytes_to_floats(&data[4], &vy);
    if (!isfinite(vx) || !isfinite(vy))
        return;
    NUC_UpdateChassis(vx, vy, data[8]);
}

static void NUC_UpdateChassis(fp32 vx, fp32 vy, uint8_t flags)
{
    nuc_instance.vx = vx;
    nuc_instance.vy = vy;
    nuc_instance.chassis_spin = flags & 0x01;
    nuc_instance.gimbal_scan = (flags >> 1) & 0x01;
    nuc_instance.use_cap = (flags >> 2) & 0x01;
    nuc_chassis_rx_tick = HAL_GetTick();
    nuc_chassis_received = true;
}

bool NUC_GetChassisCommand(fp32* vx, fp32* vy, fp32* vw, uint8_t* spin)
{
    // 通信中断更新指令，控制任务须一次读到同一帧的速度和标志位。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (nuc_chassis_received && HAL_GetTick() - nuc_chassis_rx_tick >= NUC_CHASSIS_TIMEOUT_MS)
        nuc_chassis_received = false;

    const bool fresh = nuc_chassis_received;
    if (fresh)
    {
        *vx = nuc_instance.vx;
        *vy = nuc_instance.vy;
        *vw = nuc_instance.vw;
        *spin = nuc_instance.chassis_spin;
    }
    __set_PRIMASK(primask);
    return fresh;
}

static void NUC_Gimbal_Handler(const uint8_t* data, const uint16_t len)
{
    if (len < 4)
        return;

    //unpack_4bytes_to_floats(&data[0], &nuc_instance.gimbal_yaw);原本nuc走can
    
}

void NUC_Referee_Tran(const uint8_t* data, const uint16_t len)
{
    if (!nuc_instance.init || nuc_instance.nuc_uart.handle == NULL || len < 5)
        return;

    BSP_UART_Transmit(&nuc_instance.nuc_uart, data, len, 100);
}

nuc_ctrl_t *Get_NUC_Ctrl_Instance(void)
{
    return &nuc_instance;
}
