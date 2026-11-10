#ifndef STANDARD_ROBOT_C_NX_H
#define STANDARD_ROBOT_C_NX_H
#include "can/bsp_can.h"
#include "typedef.h"

typedef enum
{
    CMD_ID_POS_CTRL = 0x400,
    CMD_ID_POS_CTRL_FIRE = 0x401,
    CMD_ID_POS_INFO = 0x501,
    CMD_ID_VEC_INFO = 0x502,
} nx_cmd_id_t;

typedef struct
{
    CAN_Instance_t* nx_can;
    fp32 target_x, target_y;
    uint8_t fire;
    fp32 target_yaw;
    fp32 target_pitch;
} nx_ctrl_t;

void NX_Init(nx_ctrl_t *nx_ctrl, CAN_HandleTypeDef *hcan);
void NX_SendPos(const nx_ctrl_t *nx_ctrl_ptr, fp32 x, fp32 y, fp32 yaw, fp32 pitch);
void NX_SendVec(const nx_ctrl_t *nx_ctrl_ptr, fp32 vx, fp32 vy);

#endif //STANDARD_ROBOT_C_NX_H