#ifndef STANDARD_ROBOT_C_DBUS_H
#define STANDARD_ROBOT_C_DBUS_H
#include "typedef.h"
#include "usart/bsp_uart.h"
#include "can/bsp_can.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_VAL_MAX 660.0f
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01 << 3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01 << 4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01 << 5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01 << 6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01 << 7)
#define RC_FRAME_LENGTH 18u
#define DBUS_MAX_LEN     (36)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3
#define DBUS_RXID        0x602

typedef enum
{
    RC_DIRECT = 0,
    RC_CAN = 1
} RC_MODE;
typedef struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1;
    uint8_t s2;
    int16_t roll;
    RC_MODE mode;
} rc_data_t;

typedef struct
{
    rc_data_t rc_data;
    UART_Instance_t dbus_usart;
    CAN_Instance_t* dbus_can;
    uint64_t last_online;
} rc_instance;

void dbus_init(rc_instance* rc_ins, RC_MODE mode, CAN_HandleTypeDef* hcan);

#endif //STANDARD_ROBOT_C_DBUS_H