#ifndef STANDARD_ROBOT_C_GM6020_H
#define STANDARD_ROBOT_C_GM6020_H
#include "can/bsp_can.h"

#define GM6020_TX_1 0x1FF
#define GM6020_TX_2 0x2FF
#define GM6020_RX_1 0x205
#define GM6020_RX_2 0x209
#define GM6020_MAXNUM_1 4
#define GM6020_MAXNUM_2 3

#define GM6020_VOLTAGGE_MAX 25000
#define GM6020_CURRENT_MAX 16384
#define GM6020_ECD_MAX 8191

typedef enum
{
    GM6020_OK = 0,
    GM6020_ERROR_INVALID_PARAM = 1,
    GM6020_ALREADY_INITIALIZED = 2,
    GM6020_ERROR = 3
} GM6020_Status_t;

typedef struct
{
    uint16_t ecd;
    int16_t speed;
    int16_t current;
    int16_t temperature;
    int16_t last_ecd;
}gm6020_ecd_t;

typedef struct
{
    CAN_Instance_t *can_ins;
    uint8_t num;
    uint32_t txid;
    gm6020_ecd_t ecd[4];
}gm6020_instance;

GM6020_Status_t gm6020_init(gm6020_instance* gm6020_ins, CAN_HandleTypeDef *hcan, uint32_t txid, uint8_t num);
GM6020_Status_t gm6020_ctrl_voltage(const gm6020_instance* gm6020_ins, int16_t voltage[4]);
GM6020_Status_t gm6020_ctrl_current(const gm6020_instance* gm6020_ins, int16_t current[4]);

#endif //STANDARD_ROBOT_C_GM6020_H