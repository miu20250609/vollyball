/* 配置文件，给keil编译使用，使用cmake编译时在CMakeLists里有相同宏定义 */
#ifndef STANDARD_ROBOT_C_ROBOT_CONFIG_H
#define STANDARD_ROBOT_C_ROBOT_CONFIG_H

// NUC 控制链路：1 = CAN2，0 = USART6。修改后重新编译。
#ifndef NUC_USE_CAN
#define NUC_USE_CAN 1
#endif

#ifndef MADE_BY_CMAKE

//#define USE_CBOARD_IMU
#ifndef CHASSIS
#define CHASSIS
#endif
//#define GIMBAL
#ifndef OMNI
#define OMNI
#endif

#endif

#endif //STANDARD_ROBOT_C_ROBOT_CONFIG_H