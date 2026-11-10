#ifndef STANDARD_ROBOT_C_M3508_H
#define STANDARD_ROBOT_C_M3508_H
#include "can/bsp_can.h"

#define M3508_TX_1 0x200
#define M3508_TX_2 0x1FF
#define M3508_RX_1 0x201
#define M3508_RX_2 0x205
#define M3508_MAXNUM 4

#define M3508_CURRENT_MAX 16384
#define M3508_ECD_MAX 8191

typedef enum
{
    M3508_OK = 0,
    M3508_ERROR_INVALID_PARAM = 1,
    M3508_ALREADY_INITIALIZED = 2,
    M3508_ERROR = 3
} M3508_Status_t;

typedef struct
{
    uint16_t ecd;
    int16_t speed;
    int16_t current;
    int16_t temperature;
    int16_t last_ecd;
}m3508_ecd_t;

typedef struct
{
    CAN_Instance_t *can_ins;
    uint8_t num;
    uint32_t txid;
    m3508_ecd_t ecd[4];
}m3508_instance;

M3508_Status_t m3508_init(m3508_instance* m3508_ins, CAN_HandleTypeDef *hcan, uint32_t txid, uint8_t num);
M3508_Status_t m3508_ctrl(const m3508_instance* m3508_ins, int16_t current[4]);

#endif //STANDARD_ROBOT_C_M3508_H