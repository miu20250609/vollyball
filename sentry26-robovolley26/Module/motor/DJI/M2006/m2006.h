#ifndef STANDARD_ROBOT_C_M2006_H
#define STANDARD_ROBOT_C_M2006_H
#include "can/bsp_can.h"

#define M2006_TX_1 0x200
#define M2006_TX_2 0x1FF
#define M2006_RX_1 0x201
#define M2006_RX_2 0x205
#define M2006_MAXNUM 4

#define M2006_CURRENT_MAX 16384
#define M2006_ECD_MAX 8191

typedef enum
{
    M2006_OK = 0,
    M2006_ERROR_INVALID_PARAM = 1,
    M2006_ALREADY_INITIALIZED = 2,
    M2006_ERROR = 3
} M2006_Status_t;

typedef struct
{
    uint16_t ecd;
    int16_t speed;
    int16_t current;
    int16_t temperature;
    int16_t last_ecd;
}m2006_ecd_t;

typedef struct
{
    CAN_Instance_t *can_ins;
    uint8_t num;
    uint32_t txid;
    m2006_ecd_t ecd[4];
}m2006_instance;

M2006_Status_t m2006_init(m2006_instance* m2006_ins, CAN_HandleTypeDef *hcan, uint32_t txid, uint8_t num);
M2006_Status_t m2006_ctrl(const m2006_instance* m2006_ins, int16_t current[4]);

#endif //STANDARD_ROBOT_C_M2006_H