#include "nx.h"

void NX_CtrlCallback(const uint8_t *data, uint32_t id, void *arg);
void NX_Init(nx_ctrl_t *nx_ctrl, CAN_HandleTypeDef *hcan)
{
    if (nx_ctrl == NULL || hcan == NULL)
        return;
    memset(nx_ctrl, 0, sizeof(nx_ctrl_t));
    nx_ctrl->nx_can = BSP_CAN_Init(hcan);
    BSP_CAN_RegisterStdCallback(nx_ctrl->nx_can, CMD_ID_POS_CTRL, NX_CtrlCallback, nx_ctrl);
    BSP_CAN_RegisterStdCallback(nx_ctrl->nx_can, CMD_ID_POS_CTRL_FIRE, NX_CtrlCallback, nx_ctrl);
}

void NX_CtrlCallback(const uint8_t *data, const uint32_t id, void *arg)
{
    if (data == NULL || arg == NULL)
        return;
    nx_ctrl_t *nx_ctrl = (nx_ctrl_t *)arg;
    nx_ctrl->fire = id & 0x01;
    const int16_t x_int = (int16_t)(data[0] | (data[1] << 8));
    const int16_t y_int = (int16_t)(data[2] | (data[3] << 8));
    const int16_t yaw_int = (int16_t)(data[4] | (data[5] << 8));
    const int16_t pitch_int = (int16_t)(data[6] | (data[7] << 8));
    nx_ctrl->target_x = (float)x_int / 1000.0f;     // 转换为米
    nx_ctrl->target_y = (float)y_int / 1000.0f;     // 转换为米
    nx_ctrl->target_yaw = (float)yaw_int / 1000.0f;   // 转换为弧度
    nx_ctrl->target_pitch = (float)pitch_int / 1000.0f; // 转换为弧度
}

void NX_SendPos(const nx_ctrl_t *nx_ctrl_ptr, const fp32 x, const fp32 y, const fp32 yaw, const fp32 pitch)
{
    const int16_t x_int = (int16_t)(x * 1000.0f);
    const int16_t y_int = (int16_t)(y * 1000.0f);
    const int16_t yaw_int = (int16_t)(yaw * 1000.0f);
    const int16_t pitch_int = (int16_t)(pitch * 1000.0f);
    uint8_t data[8];
    data[0] = x_int & 0xFF;
    data[1] = (x_int >> 8) & 0xFF;
    data[2] = y_int & 0xFF;
    data[3] = (y_int >> 8) & 0xFF;
    data[4] = yaw_int & 0xFF;
    data[5] = (yaw_int >> 8) & 0xFF;
    data[6] = pitch_int & 0xFF;
    data[7] = (pitch_int >> 8) & 0xFF;
    BSP_CAN_Transmit(nx_ctrl_ptr->nx_can, CMD_ID_POS_INFO, CAN_ID_STD, data, 8);
}

void NX_SendVec(const nx_ctrl_t *nx_ctrl_ptr, const fp32 vx, const fp32 vy)
{
    uint8_t data[8];
    memcpy(&data[0], &vx, sizeof(fp32));
    memcpy(&data[4], &vy, sizeof(fp32));
    BSP_CAN_Transmit(nx_ctrl_ptr->nx_can, CMD_ID_VEC_INFO, CAN_ID_STD, data, 8);
}