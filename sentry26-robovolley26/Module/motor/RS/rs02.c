// FUCK 灵足
#include "rs02.h"

void get_rs02_measure_mit(const uint8_t* rx_data, uint32_t id, void* arg);
void get_rs02_measure_private(const uint8_t* rx_data, uint32_t id, void* arg);
RS02_Status_t rs02_init(rs02_instance* rs02_ins, CAN_HandleTypeDef *hcan, const uint8_t motorid, const uint8_t masterid,
    const uint8_t mode, const uint8_t protocol)
{
    rs02_ins->can_ins = BSP_CAN_Init(hcan);
    rs02_ins->motorid = motorid;
    rs02_ins->masterid = masterid;
    rs02_ins->protocol = protocol;

    if (protocol == RS02_PROTOCOL_MIT)
    {
        // 这个主机id的设置莫名其妙，MIT协议下回调函数的接收id是主机id而不是电机id
        // 和bsp_can的代码相矛盾，所以我打算改为一个电机id对应一个主机id
        rs02_change_masterid_mit(rs02_ins, masterid);
        BSP_CAN_RegisterStdCallback(rs02_ins->can_ins, masterid, get_rs02_measure_mit, &rs02_ins->ecd);
        rs02_setmode_mit(rs02_ins, mode); // 还有超绝必须先设置模式再使能，又要连发两条can
        osDelay(1);
        rs02_enable_mit(rs02_ins);
        osDelay(1);
    }
    else if (protocol == RS02_PROTOCOL_PRIVATE)
    {
        // 主机id默认是0xFD
        // 私有协议用扩展can，所有控制指令都通过写入参数实现，除了运控模式一个控制指令需要发两个can，不知道这sb电机的通信协议是怎么设计的
        rs02_set_ctrlmode_private(rs02_ins, mode); // 依旧先设置模式再使能
        osDelay(1);
        BSP_CAN_RegisterExtCallback(rs02_ins->can_ins, ((RS02_CMD_CALLBACK << 24) | (motorid << 8) | masterid), 0x1F00FFFF, get_rs02_measure_private, &rs02_ins->ecd);
        rs02_enable_private(rs02_ins);
        osDelay(1);
    }
    return RS02_OK;
}

void rs02_change_masterid_mit(rs02_instance* rs02_ins, const uint8_t masterid)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_MIT)
        return;

    rs02_ins->masterid = masterid;
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x01};
    tx_data[6] = masterid;
    BSP_CAN_Transmit(rs02_ins->can_ins, rs02_ins->motorid, CAN_ID_STD, tx_data, 8);
}

void rs02_setmode_mit(const rs02_instance* rs02_ins, const uint8_t mode)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_MIT)
        return;

    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC};
    tx_data[6] = mode;
    BSP_CAN_Transmit(rs02_ins->can_ins, rs02_ins->motorid, CAN_ID_STD, tx_data, 8);
}

void rs02_enable_mit(const rs02_instance* rs02_ins)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_MIT)
        return;

    const uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    BSP_CAN_Transmit(rs02_ins->can_ins, rs02_ins->motorid, CAN_ID_STD, tx_data, 8);
}

void rs02_disable_mit(const rs02_instance* rs02_ins)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_MIT)
        return;

    const uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    BSP_CAN_Transmit(rs02_ins->can_ins, rs02_ins->motorid, CAN_ID_STD, tx_data, 8);
}

void rs02_setzero_mit(const rs02_instance* rs02_ins)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_MIT)
        return;

    const uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    BSP_CAN_Transmit(rs02_ins->can_ins, rs02_ins->motorid, CAN_ID_STD, tx_data, 8);
}

void rs02_ctrl_pos_mit(const rs02_instance* rs02_ins, const fp32 angle, const fp32 speed)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_MIT)
        return;

    uint8_t tx_data[8] = {0};
    memcpy(tx_data, &angle, 4);
    memcpy(tx_data + 4, &speed, 4);
    BSP_CAN_Transmit(rs02_ins->can_ins, rs02_ins->motorid, CAN_ID_STD, tx_data, 8);
}

void rs02_ctrl_speed_mit(const rs02_instance* rs02_ins, const fp32 speed, const fp32 current)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_MIT)
        return;

    uint8_t tx_data[8] = {0};
    memcpy(tx_data, &speed, 4);
    memcpy(tx_data + 4, &current, 4);
    BSP_CAN_Transmit(rs02_ins->can_ins, rs02_ins->motorid, CAN_ID_STD, tx_data, 8);
}

// 更换通信协议后需要重新上电
void rs02_change_protocol_mit(const rs02_instance* rs02_ins, const uint8_t protocol)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_MIT)
        return;

    const uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, protocol, 0xFD};
    BSP_CAN_Transmit(rs02_ins->can_ins, rs02_ins->motorid, CAN_ID_STD, tx_data, 8);
}

void get_rs02_measure_mit(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    rs02_ecd_t *ecd = (rs02_ecd_t*)arg;
    ecd->motorid = rx_data[0];
    ecd->ecd = (rx_data[1] << 8) | rx_data[2];
    ecd->speed = (rx_data[3] << 4) | ((rx_data[4] >> 4) & 0x0F);
    ecd->torch = ((rx_data[4] & 0x0F) << 8) | rx_data[5];
    ecd->temperature = rx_data[6] << 8 | rx_data[7];
}

void rs02_enable_private(const rs02_instance* rs02_ins)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_PRIVATE)
        return;

    const uint32_t txid = ((RS02_CMD_ENABLE << 24) | (rs02_ins->masterid << 8) | rs02_ins->motorid);
    const uint8_t tx_data[8] = {0};
    BSP_CAN_Transmit(rs02_ins->can_ins, txid, CAN_ID_EXT, tx_data, 8);
}

void rs02_disable_private(const rs02_instance* rs02_ins, const uint8_t clear_error)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_PRIVATE)
        return;

    const uint32_t txid = ((RS02_CMD_DISABLE << 24) | (rs02_ins->masterid << 8) | rs02_ins->motorid);
    uint8_t tx_data[8] = {0};
    if (clear_error) tx_data[0] = 0x01;
    BSP_CAN_Transmit(rs02_ins->can_ins, txid, CAN_ID_EXT, tx_data, 8);
}

void rs02_setzero_private(const rs02_instance* rs02_ins)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_PRIVATE)
        return;
    rs02_disable_private(rs02_ins, 1);
    osDelay(1);
    const uint32_t txid = ((RS02_CMD_SETZERO << 24) | (rs02_ins->masterid << 8) | rs02_ins->motorid);
    const uint8_t tx_data[8] = {0};
    BSP_CAN_Transmit(rs02_ins->can_ins, txid, CAN_ID_EXT, tx_data, 8);
    // osDelay(1);
    // rs02_enable_private(rs02_ins);
}

void rs02_setparam_private(const rs02_instance* rs02_ins, const uint16_t param_id, const fp32 param_value, const uint8_t param_mode)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_PRIVATE)
        return;

    const uint32_t txid = ((RS02_CMD_SETPARAM << 24) | (rs02_ins->masterid << 8) | rs02_ins->motorid);
    uint8_t tx_data[8] = {0};
    memcpy(tx_data, &param_id, 2);
    if (param_mode == Set_parameter) {
        memcpy(&tx_data[4], &param_value, 4);
    } else if (param_mode == Set_mode) {
        tx_data[4] = (uint8_t)param_value;
        tx_data[5] = 0x00;
        tx_data[6] = 0x00;
        tx_data[7] = 0x00;
    }
    BSP_CAN_Transmit(rs02_ins->can_ins, txid, CAN_ID_EXT, tx_data, 8);
}

void rs02_set_ctrlmode_private(const rs02_instance* rs02_ins, const uint8_t mode)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_PRIVATE)
        return;

    rs02_setparam_private(rs02_ins, 0x7005, mode, Set_mode);
}

void rs02_ctrl_pos_private(const rs02_instance* rs02_ins, const fp32 angle, const fp32 speed)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_PRIVATE)
        return;

    rs02_setparam_private(rs02_ins, 0x7017, speed, Set_parameter);
    rs02_setparam_private(rs02_ins, 0x7016, angle, Set_parameter);
}

void rs02_ctrl_3motor_pos_private(const rs02_instance* rs02_ins1, const rs02_instance* rs02_ins2, const rs02_instance* rs02_ins3, const fp32 angle, const fp32 speed)
{
    if (rs02_ins1 == NULL || rs02_ins2 == NULL || rs02_ins3 == NULL)
        return;
    if (rs02_ins1->protocol != RS02_PROTOCOL_PRIVATE || rs02_ins2->protocol != RS02_PROTOCOL_PRIVATE || rs02_ins3->protocol != RS02_PROTOCOL_PRIVATE)
        return;

    if (rs02_ins1->ecd.ecd_offset == 0 || rs02_ins2->ecd.ecd_offset == 0 || rs02_ins3->ecd.ecd_offset == 0)
        return;
    rs02_setparam_private(rs02_ins1, 0x7017, speed, Set_parameter);
    //osDelay(1);
    rs02_setparam_private(rs02_ins2, 0x7017, speed, Set_parameter);
    osDelay(1);
    rs02_setparam_private(rs02_ins3, 0x7017, speed, Set_parameter);
    //osDelay(1);
    rs02_setparam_private(rs02_ins1, 0x7016, angle + ((fp32)rs02_ins1->ecd.ecd_offset / RS02_UINT16_MAX * 2.0f * RS02_ECD_MAX - RS02_ECD_MAX) - 0.03f, Set_parameter);
    osDelay(1);
    rs02_setparam_private(rs02_ins2, 0x7016, angle + ((fp32)rs02_ins2->ecd.ecd_offset / RS02_UINT16_MAX * 2.0f * RS02_ECD_MAX - RS02_ECD_MAX) - 0.03f, Set_parameter);
    //osDelay(1);
    rs02_setparam_private(rs02_ins3, 0x7016, angle + ((fp32)rs02_ins3->ecd.ecd_offset / RS02_UINT16_MAX * 2.0f * RS02_ECD_MAX - RS02_ECD_MAX) - 0.03f, Set_parameter);
    osDelay(1);
    // rs02_setparam_private(rs02_ins1, 0x7016, angle, Set_parameter);
    // osDelay(1);
    // rs02_setparam_private(rs02_ins2, 0x7016, angle, Set_parameter);
    // osDelay(1);
    // rs02_setparam_private(rs02_ins3, 0x7016, angle, Set_parameter);
    // osDelay(1);
}

void rs02_ctrl_move_private(const rs02_instance* rs02_ins, const fp32 angle, const fp32 speed, const fp32 torch, const fp32 kp, const fp32 kd)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_PRIVATE)
        return;

    const uint16_t angle_uint16 = (uint16_t)((angle + RS02_ECD_MAX) * RS02_UINT16_MAX / (2 * RS02_ECD_MAX));
    const uint16_t speed_uint16 = (uint16_t)((speed + RS02_SPEED_MAX) * RS02_UINT16_MAX / (2 * RS02_SPEED_MAX));
    const uint16_t torch_uint16 = (uint16_t)((torch + RS02_TORCH_MAX) * RS02_UINT16_MAX / (2 * RS02_TORCH_MAX));
    const uint16_t kp_uint16 = (int16_t)(kp * RS02_UINT16_MAX / RS02_KP_MAX);
    const uint16_t kd_uint16 = (int16_t)(kd * RS02_UINT16_MAX / RS02_KD_MAX);
    const uint32_t txid = ((RS02_CMD_MOVECTRL << 24) | (torch_uint16 << 8) | rs02_ins->motorid);
    uint8_t tx_data[8] = {0};
    memcpy(tx_data, &angle_uint16, 2);
    memcpy(tx_data + 2, &speed_uint16, 2);
    memcpy(tx_data + 4, &kp_uint16, 2);
    memcpy(tx_data + 6, &kd_uint16, 2);
    BSP_CAN_Transmit(rs02_ins->can_ins, txid, CAN_ID_EXT, tx_data, 8);
}

// 更换通信协议后需要重新上电
void rs02_change_protocol_private(const rs02_instance* rs02_ins, const uint8_t protocol)
{
    if (rs02_ins == NULL)
        return;
    if (rs02_ins->protocol != RS02_PROTOCOL_PRIVATE)
        return;

    const uint32_t txid = ((0x19 << 24) | (rs02_ins->masterid << 8) | rs02_ins->motorid);
    const uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, protocol, protocol};
    BSP_CAN_Transmit(rs02_ins->can_ins, txid, CAN_ID_EXT, tx_data, 8);
}

void get_rs02_measure_private(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    rs02_ecd_t *ecd = (rs02_ecd_t*)arg;
    ecd->error = (id >> 16) & 0x3F;
    ecd->ecd = (rx_data[0] << 8) | rx_data[1];
    ecd->speed = (rx_data[2] << 8) | rx_data[3];
    ecd->torch = (rx_data[4] << 8) | rx_data[5];
    ecd->temperature = (rx_data[6] << 8) | rx_data[7];
    if (ecd->ecd_offset == 0)
        ecd->ecd_offset = ecd->ecd;

}