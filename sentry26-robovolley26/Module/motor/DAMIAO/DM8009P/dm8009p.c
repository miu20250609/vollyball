#include "dm8009p.h"
#include "user_lib.h"
#include <string.h>

static void dm8009p_can_rx_callback(const uint8_t *rx_data, uint32_t id, void *arg);
DM8009P_Status_t dm8009p_init(dm8009p_instance_t *ins, CAN_HandleTypeDef *hcan, const uint8_t motor_id)
{
    if (ins == NULL || hcan == NULL || motor_id == 0 || motor_id > 63)
        return DM8009P_ERROR_INVALID_PARAM;

    ins->can_ins = BSP_CAN_Init(hcan);
    if (ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    ins->motor_id = motor_id;
    ins->mode = DM8009P_MODE_MIT; 
    memset(&ins->ecd, 0, sizeof(ins->ecd));

    if (BSP_CAN_RegisterStdCallback(ins->can_ins, motor_id, dm8009p_can_rx_callback, ins) != CAN_OK)
        return DM8009P_ERROR_NOT_INIT;

    return DM8009P_OK;
}

static void dm8009p_can_rx_callback(const uint8_t *rx_data, uint32_t id, void *arg)
{
    if (rx_data == NULL || arg == NULL)
        return;

    dm8009p_instance_t *ins = (dm8009p_instance_t *)arg;
    dm8009p_ecd_t *fb = &ins->ecd;

    fb->err = (rx_data[1] >> 4) & 0x0F;
    fb->pos_raw = (uint16_t)(rx_data[2] << 8) | rx_data[3];
    fb->vel_raw = (int16_t)(((rx_data[4] << 4) | (rx_data[5] >> 4)) & 0x0FFF);
    if (fb->vel_raw & 0x0800) fb->vel_raw |= 0xF000;
    fb->tor_raw = (int16_t)(((rx_data[5] & 0x0F) << 8) | rx_data[6]);
    if (fb->tor_raw & 0x0800) fb->tor_raw |= 0xF000;
    fb->mos_temp = rx_data[7];
    fb->rotor_temp = 0;
}

DM8009P_Status_t dm8009p_set_mode(dm8009p_instance_t *ins, const DM8009P_Mode_t mode)
{
    if (ins == NULL)
        return DM8009P_ERROR_INVALID_PARAM;
    ins->mode = mode;
    return DM8009P_OK;
}

DM8009P_Status_t dm8009p_ctrl_mit(const dm8009p_instance_t *ins,
                                  const float p_des, const float v_des,
                                  const float kp, const float kd, const float t_ff)
{
    if (ins == NULL || ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    if (kp < 0 || kp > DM8009P_KP_MAX || kd < 0 || kd > DM8009P_KD_MAX)
        return DM8009P_ERROR_INVALID_PARAM;

    uint16_t p_raw = (uint16_t)((p_des / DM8009P_POS_FACTOR) + 32768);
    uint16_t v_raw = (uint16_t)((v_des / DM8009P_VEL_FACTOR) + 2048);
    uint8_t  kp_raw = (uint8_t)(kp * 255.0f / DM8009P_KP_MAX);
    uint16_t kd_raw = (uint16_t)(kd * 4095.0f / DM8009P_KD_MAX);
    uint8_t  t_raw = (uint8_t)((t_ff / DM8009P_TORQUE_FACTOR) + 128);

    uint8_t txdata[8];
    txdata[0] = (uint8_t)(p_raw >> 8);
    txdata[1] = (uint8_t)(p_raw & 0xFF);
    txdata[2] = (uint8_t)((v_raw >> 4) & 0xFF);
    txdata[3] = (uint8_t)(((v_raw & 0x0F) << 4) | ((kp_raw >> 4) & 0x0F));
    txdata[4] = (uint8_t)(((kp_raw & 0x0F) << 4) | ((kd_raw >> 8) & 0x0F));
    txdata[5] = (uint8_t)(kd_raw & 0xFF);
    txdata[6] = t_raw;
    txdata[7] = 0;

    uint32_t tx_id = DM8009P_MIT_BASE_ID + ins->motor_id;
    if (BSP_CAN_Transmit(ins->can_ins, tx_id, CAN_ID_STD, txdata, 8) != CAN_OK)
        return DM8009P_ERROR_CAN_TX;

    return DM8009P_OK;
}

/* 位置速度模式控制 */
DM8009P_Status_t dm8009p_ctrl_pos_vel(const dm8009p_instance_t *ins, const float p_des, const float v_des)
{
    if (ins == NULL || ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    uint8_t txdata[8];

    memcpy(txdata, &p_des, 4);
    memcpy(txdata + 4, &v_des, 4);

    uint32_t tx_id = DM8009P_POSVEL_BASE_ID + ins->motor_id;
    if (BSP_CAN_Transmit(ins->can_ins, tx_id, CAN_ID_STD, txdata, 8) != CAN_OK)
        return DM8009P_ERROR_CAN_TX;

    return DM8009P_OK;
}

DM8009P_Status_t dm8009p_ctrl_vel(const dm8009p_instance_t *ins, const float v_des)
{
    if (ins == NULL || ins->can_ins == NULL)
        return DM8009P_ERROR_NOT_INIT;

    uint8_t txdata[8];
    
    memcpy(txdata, &v_des, 4);
    memset(txdata + 4, 0, 4);

    const uint32_t tx_id = DM8009P_VEL_BASE_ID + ins->motor_id;
    if (BSP_CAN_Transmit(ins->can_ins, tx_id, CAN_ID_STD, txdata, 8) != CAN_OK)
        return DM8009P_ERROR_CAN_TX;

    return DM8009P_OK;
}