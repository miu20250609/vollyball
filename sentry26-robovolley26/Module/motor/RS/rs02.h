#ifndef STANDARD_ROBOT_C_RS02_H
#define STANDARD_ROBOT_C_RS02_H
#include "can/bsp_can.h"
#include "typedef.h"
#include "user_lib.h"

// 注意是将uint16_t[0, 65535]映射到[-RS02_ECD_MAX, RS02_ECD_MAX]，而不是[0, RS02_ECD_MAX], speed和torch同理
#define RS02_UINT16_MAX 65535.0f
#define RS02_ECD_MAX (4.0f*PI)
#define RS02_SPEED_MAX 44.0f
#define RS02_TORCH_MAX 17.0f
#define RS02_KP_MAX 500.0f
#define RS02_KD_MAX 5.0f
#define RS02_TEMP_RATIO 0.1f

#define Set_mode 'j'
#define Set_parameter 'p'

typedef enum
{
    RS02_MODE_MIT = 0, // MIT模式
    RS02_MODE_POS = 5, // 位置控制
    RS02_MODE_SPEED = 2, // 速度控制
} RS02_CTRL_MODE;

typedef enum
{
    RS02_PROTOCOL_PRIVATE = 0, // 私有协议
    RS02_PROTOCOL_CANOPEN = 1, // CANopen协议
    RS02_PROTOCOL_MIT = 2, // MIT协议
} RS02_PROTOCOL;

typedef enum
{
    RS02_CMD_GETID = 0x00,
    RS02_CMD_MOVECTRL = 0x01,
    RS02_CMD_CALLBACK = 0x02,
    RS02_CMD_ENABLE = 0x03,
    RS02_CMD_DISABLE = 0x04,
    RS02_CMD_SETZERO = 0x06,
    RS02_CMD_SETID = 0x07,
    RS02_CMD_GETPARAM = 0x11,
    RS02_CMD_SETPARAM = 0x12,
    RS02_CMD_ERRORCB = 0x15,
    RS02_CMD_SAVEPARAM = 0x16,
    RS02_CMD_CHANGEBUAD = 0x17,
    RS02_CMD_AUTOFEEDBACK = 0x18,
    RS02_CMD_CHANGEPROTOCOL = 0x19
} RS02_CMD_PRIVATE;

typedef enum
{
    RS02_OK = 0,
    RS02_ERROR_INVALID_PARAM = 1,
    RS02_ALREADY_INITIALIZED = 2,
    RS02_ERROR = 3
} RS02_Status_t;

typedef struct
{
    uint8_t motorid;
    uint8_t error;
    uint16_t ecd;
    uint16_t speed;
    uint16_t torch;
    uint16_t temperature;
    uint16_t last_ecd;
    uint16_t ecd_offset;
}rs02_ecd_t;

typedef struct
{
    CAN_Instance_t *can_ins;
    uint8_t motorid;
    uint8_t masterid;
    rs02_ecd_t ecd;
    RS02_CTRL_MODE mode;
    RS02_PROTOCOL protocol;
}rs02_instance;

RS02_Status_t rs02_init(rs02_instance* rs02_ins, CAN_HandleTypeDef *hcan, uint8_t motorid,
    uint8_t masterid, uint8_t mode, uint8_t protocol);
void rs02_change_masterid_mit(rs02_instance* rs02_ins, uint8_t masterid);
void rs02_setmode_mit(const rs02_instance* rs02_ins, uint8_t mode);
void rs02_enable_mit(const rs02_instance* rs02_ins);
void rs02_disable_mit(const rs02_instance* rs02_ins);
void rs02_setzero_mit(const rs02_instance* rs02_ins);
void rs02_ctrl_pos_mit(const rs02_instance* rs02_ins, fp32 angle, fp32 speed);
void rs02_ctrl_speed_mit(const rs02_instance* rs02_ins, fp32 speed, fp32 current);
void rs02_change_protocol_mit(const rs02_instance* rs02_ins, uint8_t protocol);
void rs02_enable_private(const rs02_instance* rs02_ins);
void rs02_disable_private(const rs02_instance* rs02_ins, uint8_t clear_error);
void rs02_setzero_private(const rs02_instance* rs02_ins);
void rs02_set_ctrlmode_private(const rs02_instance* rs02_ins, uint8_t mode);
void rs02_ctrl_pos_private(const rs02_instance* rs02_ins, fp32 angle, fp32 speed);
void rs02_ctrl_3motor_pos_private(const rs02_instance* rs02_ins1, const rs02_instance* rs02_ins2, const rs02_instance* rs02_ins3, fp32 angle, fp32 speed);
void rs02_ctrl_move_private(const rs02_instance* rs02_ins, fp32 angle, fp32 speed, fp32 torch, fp32 kp, fp32 kd);
void rs02_change_protocol_private(const rs02_instance* rs02_ins, uint8_t protocol);
#endif //STANDARD_ROBOT_C_RS02_H