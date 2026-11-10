#include "task_init.h"
#include "cmsis_os.h"
#include "robovolley.h"
#include "daemon.h"
osThreadId control_taskHandle;
osThreadId daemon_taskHandle;
void task_init()
{
    osThreadDef(ControlTask, Control_Task, osPriorityNormal, 0, 256);
    control_taskHandle = osThreadCreate(osThread(ControlTask), NULL);

    osThreadDef(DaemonTask, Daemon_Task, osPriorityLow, 0, 128);
    daemon_taskHandle = osThreadCreate(osThread(DaemonTask), NULL);

}