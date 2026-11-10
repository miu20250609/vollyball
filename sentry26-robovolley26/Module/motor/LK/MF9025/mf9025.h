#ifndef STANDARD_ROBOT_C_MF9025_H
#define STANDARD_ROBOT_C_MF9025_H
#include "can/bsp_can.h"
#include "typedef.h"

#define MF9025_TX_MIN 0x141
#define MF9025_TX_MAX 0x160

#define MF9025_ECD_MAX 65535
#define MF9025_ECD_IN_ZERO 0x4520

#define MF9025_MAX_IQ 2048

typedef enum
{
    MF9025_OK = 0,
    MF9025_ERROR_INVALID_PARAM = 1,
    MF9025_ALREADY_INITIALIZED = 2,
    MF9025_ERROR = 3
} MF9025_Status_t;

typedef enum
{
    CMD_9025_INCREMENT_ANGLE_CONTROL = 0xA8,
    CMD_9025_ANGLE_CONTROL = 0xA6,
    CMD_9025_SPEED_CONTROL = 0xA2,
    CMD_9025_READ_MEASURE  = 0x9C,
    CMD_9025_READ_ENCODER = 0x90,
    CMD_9025_READ_CONTROL_PARAM = 0xC0,
    CMD_9025_WRITE_CONTROL_PARAM = 0xC1
} MF9025_CMD_t;

typedef enum
{
    PARAM_9025_ANGLE_PID = 0x0A,
    PARAM_9025_SPEED_PID = 0x0B,
    PARAM_9025_IQ_PID = 0x0C
} MF9025_PARAM_t;

typedef struct
{
    uint16_t ecd;
    int16_t speed; // 1dps/LSB
    int16_t iq;
    int8_t temperature;
    int16_t last_ecd;

    uint16_t ecd_offset; // 记录初始编码器值用于后续计算相对初始位置角度
    uint16_t zero_offset; // 记录初始编码器值与绝对零位的差值
}mf9025_ecd_t;

typedef struct
{
    CAN_Instance_t *can_ins;
    uint32_t txid;
    mf9025_ecd_t ecd;
}mf9025_instance;

MF9025_Status_t mf9025_init(mf9025_instance* mf9025_ins, CAN_HandleTypeDef *hcan, uint32_t txid);
void mf9025_speed_init(const mf9025_instance* mf9025_ins, const uint16_t pid_v[3]);
MF9025_Status_t mf9025_ctrl_speed(const mf9025_instance* mf9025_ins, uint16_t iqControl, uint32_t speedControl);
MF9025_Status_t mf9025_set_pid(const mf9025_instance* mf9025_ins, uint8_t param, uint16_t kp, uint16_t ki, uint16_t kd);
MF9025_Status_t mf9025_get_measure(const mf9025_instance* mf9025_ins);

#endif //STANDARD_ROBOT_C_MF9025_H