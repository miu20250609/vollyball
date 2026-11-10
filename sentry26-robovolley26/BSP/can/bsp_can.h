#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* CAN配置 */
#define MAX_CAN_INSTANCES          2U          /* 支持的最大CAN实例数（CAN1, CAN2） */
#define MAX_CAN_FILTERS           14U          /* 每个CAN支持的最大过滤器数量 */
#define CAN_TX_BUFFER_SIZE        10U          /* 发送缓冲区大小 */
#define CAN_RX_BUFFER_SIZE        20U          /* 接收缓冲区大小 */
#define CAN_TASK_STACK_SIZE       128U         /* CAN任务栈大小 */

/* CAN初始化结果 */
typedef enum {
    CAN_OK = 0,                    /* 成功 */
    CAN_ERROR = 1,                 /* 错误 */
    CAN_ERROR_INVALID_PARAM = 2,   /* 无效参数 */
    CAN_ERROR_NO_MEMORY = 3        /* 内存不足 */
} CAN_Status_t;

/* CAN过滤器配置结构 */
typedef struct {
    uint32_t filter_id;            /* 过滤器ID */
    uint32_t filter_mask;          /* 过滤器掩码 */
    uint32_t id_type;              /* ID类型 */
    uint8_t filter_bank;           /* 过滤器组号 */
} CAN_Filter_t;

/* 接收回调函数指针类型 */
typedef void (*CAN_RxCallback_t)(const uint8_t *msg, uint32_t id, void* arg);

/* 错误回调函数指针类型 */
typedef void (*CAN_ErrorCallback_t)(uint32_t error);

/* CAN实例句柄结构 */
typedef struct CAN_Handle_s CAN_Instance_t;

struct CAN_Handle_s {
    CAN_HandleTypeDef *handle;                      /* HAL CAN句柄 */
    CAN_RxCallback_t rx_callbacks[MAX_CAN_FILTERS+1]; /* 接收回调函数数组 */
    uint8_t rxbuf[8];                               /* 接收缓冲区 */
    uint32_t callback_ids[MAX_CAN_FILTERS+1];         /* 回调对应的CAN ID */
    void* callback_args[MAX_CAN_FILTERS+1];        /* 回调对应保存数据位置 */
    uint32_t callback_idmasks[MAX_CAN_FILTERS+1];     /* 回调对应的ID掩码 */
    uint32_t callback_ide[MAX_CAN_FILTERS+1];       /* 回调对应的ID类型 */
    uint8_t can_instance;                           /* CAN实例号 (0或1) */
    uint8_t init;                                   /* CAN是否完成初始化*/
};

/* 函数原型 */
CAN_Instance_t* BSP_CAN_Init(CAN_HandleTypeDef *hcan);

CAN_Status_t BSP_CAN_RegisterStdCallback(CAN_Instance_t *can_ins,
                                     uint32_t can_id,
                                     CAN_RxCallback_t rxCallback,
                                     void* arg);

CAN_Status_t BSP_CAN_RegisterExtCallback(CAN_Instance_t *can_ins,
                                     uint32_t can_id,
                                     uint32_t id_mask,
                                     CAN_RxCallback_t rxCallback,
                                     void* arg);

CAN_Status_t BSP_CAN_Transmit(const CAN_Instance_t *can_ins,
                                uint32_t can_id,
                                uint32_t id_type,
                                const uint8_t *data,
                                uint8_t dlc);

void BSP_CAN_Rx_IRQHandler(CAN_Instance_t *can_ins);

#endif /* BSP_CAN_H */