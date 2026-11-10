#ifndef STANDARD_ROBOT_C_DAEMON_H
#define STANDARD_ROBOT_C_DAEMON_H
#include "LED/led.h"

typedef struct
{
    led_instance led_ins;
} daemon_t;

void Daemon_Init(void);
void Daemon_Task(void const * argument);

#endif //STANDARD_ROBOT_C_DAEMON_H