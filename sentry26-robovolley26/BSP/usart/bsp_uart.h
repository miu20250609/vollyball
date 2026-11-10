#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* 缓冲区配置 */
#define RX_BUFFER_SIZE          128U
#define TX_BUFFER_SIZE          128U
#define MAX_UART_INSTANCES      4U

/* UART初始化结果 */
typedef enum {
    UART_OK = 0,                    /* 成功 */
    UART_ERROR = 1,                 /* 错误 */
    UART_ERROR_INVALID_PARAM = 2,   /* 无效参数 */
    UART_ERROR_ALREADY_INIT = 3     /* 已初始化 */
} UART_Status_t;

/* UART句柄结构前向声明 */
typedef struct UART_Handle_s UART_Instance_t;

/* 接收回调函数指针类型 */
typedef void (*UART_RxCallback_t)(uint8_t *data, uint16_t len);

/* 错误回调函数指针类型 */
typedef void (*UART_ErrorCallback_t)(uint32_t error);

/* UART句柄结构定义 */
struct UART_Handle_s {
    UART_HandleTypeDef *handle;                   /* HAL UART句柄 */
    uint8_t rxBuffer[2][RX_BUFFER_SIZE];          /* 双缓冲接收 */
    uint8_t txBuffer[TX_BUFFER_SIZE];             /* 发送缓冲区 */
    UART_RxCallback_t RxCallback;                 /* 接收完成回调函数 */
    UART_ErrorCallback_t ErrorCallback;           /* 错误回调函数（可选） */
    osSemaphoreId txSemaphoreId;                     /* 线程安全的缓冲区访问互斥量 */

    uint16_t rxBufferSize;                        /* 接收缓冲区大小 */
    uint16_t txBufferSize;                        /* 发送缓冲区大小 */
};

/* Function prototypes */
UART_Status_t BSP_UART_Init(UART_Instance_t *uart_ins,
                           UART_HandleTypeDef *huart,
                           uint32_t baudrate,
                           UART_RxCallback_t rxCallback,
                           UART_ErrorCallback_t errorCallback,
                           uint16_t txBufferSize,
                           uint16_t rxBufferSize);
UART_Status_t BSP_UART_Transmit(UART_Instance_t *uart_ins, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
void BSP_UART_IRQHandler(const UART_Instance_t *uart_ins);
UART_Instance_t* get_uart_ins_map(uint8_t uart);
#endif /* BSP_UART_H */