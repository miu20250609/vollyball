#include "module_init.h"
#include "can.h"
#include "usart.h"
#include "cmsis_os.h"
#include "dwt/bsp_dwt.h"
#include "motor/DJI/M3508/m3508.h"
#include "motor/RS/rs02.h"
#include "DBUS/dbus.h"
#include "IMU/BMI088/BMI088driver.h"
#include "IMU/INS/ins.h"
#include "Odom/odom.h"
#include "NX/nx.h"

void Modules_Init(void)
{
    DWT_Init(DWT_CLOCK_FREQ);
    dbus_init(RC_DIRECT, &hcan1);
    BMI088_Init();
    INS_Init();
    m3508_init(&hcan1, M3508_TX_1, 3);
    for (uint8_t i = 0; i < 5; i++)
    {
        rs02_init(&hcan2, i + 0x01, 0xFD, RS02_MODE_POS, RS02_PROTOCOL_PRIVATE);
        osDelay(1);
    }
    Odom_Init(&huart1);
    NX_Init(&hcan2);
}
