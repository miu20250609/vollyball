#ifndef STANDARD_ROBOT_C_VESC_H
#define STANDARD_ROBOT_C_VESC_H
#include "typedef.h"
#include "can/bsp_can.h"

#define VESC_CNTMAX 5

typedef enum {
    /* ========== 电机控制 ========== */
    CAN_PACKET_SET_DUTY                      = 0,   // 占空比 ×1e5
    CAN_PACKET_SET_CURRENT                    = 1,   // 电机电流 ×1e3
    CAN_PACKET_SET_CURRENT_BRAKE              = 2,   // 刹车电流 ×1e3
    CAN_PACKET_SET_RPM                        = 3,   // eRPM, 不放大
    CAN_PACKET_SET_POS                        = 4,   // 位置 ±180° ×1e6

    /* ========== 状态广播类 ========== */
    CAN_PACKET_STATUS                         = 9,   // eRPM/电机电流/占空比
    CAN_PACKET_SET_CURRENT_REL                = 10,  // 电流% ×1e5
    CAN_PACKET_SET_CURRENT_BRAKE_REL          = 11,  // 刹车电流% ×1e5
    CAN_PACKET_SET_CURRENT_HANDBRAKE          = 12,  // 驻车电流 ×1e3, 需编码器
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL      = 13,  // 驻车电流% ×1e5
    CAN_PACKET_STATUS_2                       = 14,  // 放电Ah/充电Ah
    CAN_PACKET_STATUS_3                       = 15,  // 放电Wh/充电Wh
    CAN_PACKET_STATUS_4                       = 16,  // FET temp / motor temp / bat I / pos
    CAN_PACKET_STATUS_5                       = 27
} CAN_PACKET_ID;

typedef struct
{
    int16_t duty;
    int16_t current;
    int32_t erpm;
}vesc_packet_1;

typedef struct
{
    vesc_packet_1 packet_1;
}vesc_packet_t;

typedef struct
{
    CAN_Instance_t* can_instance;
    vesc_packet_t packet;
    uint32_t mcuid;
    uint8_t init;
}vesc_instance_t;
void vesc_init(uint32_t mcuid, CAN_HandleTypeDef *hcan);
void vesc_current_ctrl(const vesc_instance_t* vesc_ins, fp32 current);
void vesc_speed_ctrl(const vesc_instance_t* vesc_ins, int32_t erpm);
void vesc_pos_ctrl(const vesc_instance_t* vesc_ins, fp32 angle);
vesc_instance_t* Get_Vesc_Ptr(uint32_t mcuid);

#endif //STANDARD_ROBOT_C_VESC_H