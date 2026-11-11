#include "robovolley.h"
#include <string.h>
#include "arm_math.h"
#include "can.h"
#include "usart.h"
#include "user_lib.h"
#include "pid.h"
#include "dwt/bsp_dwt.h"
//#include "NX/nx.h" // 排球功能相关，暂时注释掉

const fp32 filter_speed_alpha[3] = {0.2f, 0.2f, 0.2f};
const pid_config m3508_pid_config = {.mode = PID_POSITION, .kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config chassisyaw_pid_config = {.mode = PID_POSITION, .kp = 10.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 20000.0f, .max_iout = 5000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config bmi088_v_pid_config = {.mode = PID_POSITION, .kp = 10.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 20000.0f, .max_iout = 5000.0f,};
robot_t chassis;
#define MAX_OF_THREE(a, b, c) (((a) > (b)) ? (((a) > (c)) ? (a) : (c)) : (((b) > (c)) ? (b) : (c)))

// ---------- 内部静态函数声明 ----------
static void chassis_filter_init(robot_t* chassis_ptr);
static void chassis_pid_init(robot_t* chassis_ptr);
static void chassis_switch_controller(robot_t* chassis_ptr);
static void chassis_calc_move_speed(robot_t* chassis_ptr);
static void chassis_calc_wheelmotor_speed(robot_t* chassis_ptr);
static void chassis_calc_wheelmotor_pidout(robot_t* chassis_ptr);
static void chassis_send_wheelmotor_cmd(const robot_t* chassis_ptr);
static void chassis_test_spin(const robot_t* chassis_ptr); // 测试模式：PID电流环驱动电机
//static void rc_offline_process(robot_t* chassis_ptr);
// ---- 以下为排球机构功能，暂时注释掉 ----
//static void gimbal_calc_pitch_target(robot_t* chassis_ptr);
//static void gimbal_calc_shoot_target(robot_t* chassis_ptr);
//static void gimbal_shoot_process_1(robot_t* chassis_ptr);
//static void gimbal_shoot_process_2(robot_t* chassis_ptr);
//static void gimbal_shoot_process_3(robot_t* chassis_ptr);
//static void chasssis_send_pos(const robot_t* chassis_ptr);
//static void chasssis_send_vec(const robot_t* chassis_ptr);
//static void chasssis_send_motorspeed(const robot_t* chassis_ptr);

//void rs02_send_cmd_task(const void* argument);
//osThreadId rs02_taskHandle;

static void rc_offline_process(robot_t* chassis_ptr); // 遥控器离线处理（1.5s无信号强制停车）
/**
 * @brief 底盘初始化函数
 *        初始化电机控制、遥控器、雷达通信等
 */
void Chassis_Init(robot_t* chassis_ptr)
{
    DWT_Init(DWT_CLOCK_FREQ);
    dbus_init(&chassis_ptr->rc, RC_DIRECT, &hcan1);
    m3508_init(&chassis_ptr->m3508, &hcan1, M3508_TX_1, 3);

    // 雷达（NUC上位机）通信初始化，与哨兵工程一致挂在huart6上
    NUC_Init(&huart6);
    chassis_ptr->nuc_ctrl = Get_NUC_Ctrl_Instance();

    // ---- 以下为排球机构相关初始化，暂时注释掉 ----
    static IMU_Data_t bmi088;
    BMI088_Init(&bmi088);
    INS_Init(&chassis.ins, &bmi088);
    // for (uint8_t i = 0; i < 5; i++)
    // {
    //     rs02_init(&chassis_ptr->rs02[i], &hcan2, i + 0x01, 0xFD, RS02_MODE_POS, RS02_PROTOCOL_PRIVATE);
    //     osDelay(1);
    // }
    // osDelay(1);
    // vesc_init(121, &hcan1);
    // chassis_ptr->vesc = Get_Vesc_Ptr(121);
    // Odom_Init(&chassis_ptr->odom, &huart1);
    // NX_Init(&chassis_ptr->nx_ctrl, &hcan2);
    // hc_init(GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_12);
    // chassis_ptr->hc = Get_HC_Ptr();
    vofa_init(&huart1, 7, 0); // VOFA 转发：huart1（PA9=TX, PA10=RX）接正点原子无线烧录器串口透传
    chassis_ptr->vofa = Get_Vofa_Ptr();

    memset(&chassis_ptr->ctrl, 0, sizeof(chassis_ptr->ctrl));
    chassis_filter_init(chassis_ptr);
    chassis_pid_init(chassis_ptr);

    // osThreadDef(RS02_ctrl_Task, rs02_send_cmd_task, osPriorityHigh, 0, 128);
    // rs02_taskHandle = osThreadCreate(osThread(RS02_ctrl_Task), chassis_ptr);
}

/**
 * @brief 底盘控制任务，供FreeRTOS调用
 */
void Control_Task(const void* argument)
{
    Chassis_Init(&chassis);
    static uint16_t ctrl_loop = 0;
    while(1)
    {
        osDelay(1);
        if (ctrl_loop % 3 == 0) // 控制循环 300Hz左右
        {
            // ===== 上电即电机测试：原地旋转（PID电流环，目标电流3000）。需要时取消注释下一行，并注释掉下面管线 =====
            // chassis_test_spin(&chassis);
            chassis_switch_controller(&chassis);       // 切换控制源（遥控/上位机等）
            rc_offline_process(&chassis);              // 处理遥控器离线情况（1.5s无信号强制停车）
            chassis_calc_move_speed(&chassis);         // 计算并过滤移动速度
            chassis_calc_wheelmotor_speed(&chassis);   // 解算底盘轮毂电机速度
            chassis_calc_wheelmotor_pidout(&chassis);  // 计算电机控制PID输出
            chassis_send_wheelmotor_cmd(&chassis);     // 下发3508控制指令
        }
        if (ctrl_loop % 10 == 0) // VOFA 转发循环 100Hz
        {
            static uint8_t vofa_link = 0;
            vofa_link ^= 1; // 链路心跳：每次发送翻转，上位机看到闪烁即链路通（雷达静默时也照常闪烁）
            // ===== 临时：遥控器调试，VOFA 前6通道 = 遥控 ch0~ch3、s1、s2（调试完删掉本块，恢复下面注释的雷达转发块） =====
            // const fp32 send[7] = {
            //     (fp32)chassis.rc.rc_data.ch0,      // 遥控 ch0（右杆左右，0=居中，范围±660）
            //     (fp32)chassis.rc.rc_data.ch1,      // 遥控 ch1（右杆上下）
            //     (fp32)chassis.rc.rc_data.ch2,      // 遥控 ch2（左杆上下=前后）
            //     (fp32)chassis.rc.rc_data.ch3,      // 遥控 ch3（左杆左右=平移）
            //     (fp32)chassis.rc.rc_data.s1,       // 拨杆 s1（上1/中3/下2）
            //     (fp32)chassis.rc.rc_data.s2,       // 拨杆 s2（上1/中3/下2）
            //     (fp32)vofa_link,                   // 链路心跳：1↔0 闪烁 = 无线链路通
            // };
            // vofa_print(send);
            // ===== 原来的雷达转发块（恢复时删除上面的临时块并取消本块注释） =====
            const fp32 send[7] = {
                chassis.nuc_ctrl->vx,                     // 雷达原始 vx (m/s)
                chassis.nuc_ctrl->vy,                     // 雷达原始 vy (m/s)
                (fp32)chassis.ctrl.chassis_spin,          // 小陀螺标志（仅UPC模式有效）
                chassis.ctrl.chassis_speed_filter[0].out, // 滤波后 vx
                chassis.ctrl.chassis_speed_filter[1].out, // 滤波后 vy
                chassis.ctrl.chassis_speed_filter[2].out, // 滤波后 vw
                (fp32)vofa_link,                          // 链路心跳：1↔0 闪烁 = 无线链路通
            };
            vofa_print(send);
        }
        // ---- 排球机构相关循环，暂时注释掉 ----
        // if (ctrl_loop % 3 == 1) // 云台控制循环 300Hz左右
        // {
        //     chassis_switch_controller(&chassis);       // 切换控制源（遥控/上位机等）
        //     gimbal_calc_pitch_target(&chassis); // 计算云台pitch轴目标位置
        //     gimbal_calc_shoot_target(&chassis); // 计算击球目标位置和发球状态
        //     gimbal_shoot_process_1(&chassis);      // 发球状态机处理
        // }
        // if (ctrl_loop % 5 == 0) // 通信循环 200Hz左右
        // {
        //     chasssis_send_pos(&chassis);
        //     chasssis_send_motorspeed(&chassis);
        // }
        // if (ctrl_loop % 5 == 1) // 通信循环 200Hz左右
        // {
        //     chasssis_send_vec(&chassis);
        //     const fp32 send[3] = {(fp32)(chassis.rs02[4].ecd.ecd - chassis.rs02[4].ecd.ecd_offset) / RS02_UINT16_MAX * 2.0f * RS02_ECD_MAX - RS02_ECD_MAX, (fp32)chassis.rs02[4].ecd.speed / RS02_UINT16_MAX * 2.0f * RS02_SPEED_MAX - RS02_SPEED_MAX, (fp32)chassis.rs02[4].ecd.torch / RS02_UINT16_MAX * 2.0f * RS02_TORCH_MAX - RS02_TORCH_MAX};
        //     vofa_print(send);
        // }
        // if (ctrl_loop % 50 == 1)
        // {
        //     hc_trig();
        // }

        ctrl_loop++;
        if (ctrl_loop > 3000) ctrl_loop = 0;
    }
}

/**
 * @brief 底盘一阶低通滤波初始化
 */
static void chassis_filter_init(robot_t* chassis_ptr)
{
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[0], CHASSIS_CONTROL_TIME, filter_speed_alpha[0]);
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[1], CHASSIS_CONTROL_TIME, filter_speed_alpha[1]);
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[2], CHASSIS_CONTROL_TIME, filter_speed_alpha[2]);
}

/**
 * @brief 初始化PID参数
 */
static void chassis_pid_init(robot_t* chassis_ptr)
{
    for (int i = 0; i < 3; i++)
        PID_init(&chassis_ptr->ctrl.m3508_controller[i].pid, m3508_pid_config);
        PID_init(&chassis_ptr->ins.pid, bmi088_v_pid_config);
}

/**
 * @brief 切换底盘控制器（通过遥控器S1拨杆）
 */
static void chassis_switch_controller(robot_t* chassis_ptr)
{
    chassis_ptr->ctrl.robot_controller = chassis_ptr->rc.rc_data.s1;
    chassis_ptr->ctrl.shoot_mode = chassis_ptr->rc.rc_data.s2;
}

/**
 * @brief 电机测试模式：三个轮子同向给定电流（PID电流环闭环），底盘原地旋转
 *        电流原始值范围 -16384~16384，目标值取 CHASSIS_TEST_CURRENT
 *        反馈为 m3508 上报的实际电流，PID 输出为下发电流指令
 */
static void chassis_test_spin(const robot_t* chassis_ptr)
{
    static pid_t test_current_pid[3];
    static uint8_t test_pid_init_done = 0;
    if (!test_pid_init_done)
    {
        const pid_config test_current_cfg = {.mode = PID_POSITION, .kp = 1.0f, .ki = 0.05f, .kd = 0.0f, .max_out = 16384.0f, .max_iout = 3000.0f,
            .out_limit_delta_P = 2000.0f, .out_limit_delta_N = 5000.0f, .deadzone = 0.0f};
        for (int i = 0; i < 3; i++)
            PID_init(&test_current_pid[i], test_current_cfg);
        test_pid_init_done = 1;
    }
    int16_t m3508_iq[4] = {0};
    for (int i = 0; i < 3; i++)
    {
        PID_calc(&test_current_pid[i], chassis_ptr->m3508.ecd[i].current, CHASSIS_TEST_CURRENT);
        m3508_iq[i] = (int16_t)test_current_pid[i].out[0];
    }
    m3508_ctrl(&chassis_ptr->m3508, m3508_iq);
}

static void rc_offline_process(robot_t* chassis_ptr)
{
    if (DWT_GetTimeline_us() - chassis_ptr->rc.last_online > ms_to_us(1500))
    {
        chassis_ptr->ctrl.robot_controller = CHASSIS_RC_OFFLINE;
        chassis_ptr->ctrl.shoot_mode = SHOOT_RECEIVE;
        chassis_ptr->ctrl.given_chassis_v[0] = 0;
        chassis_ptr->ctrl.given_chassis_v[1] = 0;
        chassis_ptr->ctrl.given_chassis_w = 0;
        chassis_ptr->ctrl.rc_offline = 1;
    }
    else
        chassis_ptr->ctrl.rc_offline = 0;
}

/**
 * @brief 解算给定移动速度（获取控制源指令并滤波）
 *        遥控器模式走DBUS摇杆，上位机模式走NUC雷达下发速度
 */
static void chassis_calc_move_speed(robot_t* chassis_ptr)
{
    fp32 vx, vy, vw;
    if (chassis_ptr->ctrl.robot_controller == CHASSIS_RC)//遥控器控制
    {
        chassis_ptr->ctrl.chassis_spin = 0;//小陀螺模式标志，仅雷达（UPC）模式有效
        chassis_ptr->ctrl.gimbal_scan = 0;

        vx = (fp32)chassis_ptr->rc.rc_data.ch2 * CHASSIS_MAX_V / RC_VAL_MAX;
        vy = (fp32)chassis_ptr->rc.rc_data.ch3 * CHASSIS_MAX_V / RC_VAL_MAX;
        vw = (fp32)chassis_ptr->rc.rc_data.ch0 * CHASSIS_MAX_W / RC_VAL_MAX;
    }
    else if (chassis_ptr->ctrl.robot_controller == CHASSIS_UPC)//上位机（雷达）控制
    {
        chassis_ptr->ctrl.chassis_spin = chassis_ptr->nuc_ctrl->chassis_spin;//是否小陀螺

        vx = loop_float_constrain(chassis_ptr->nuc_ctrl->vx, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        vy = loop_float_constrain(chassis_ptr->nuc_ctrl->vy, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        vw = loop_float_constrain(chassis_ptr->nuc_ctrl->vw, -CHASSIS_MAX_W, CHASSIS_MAX_W);

        if (chassis_ptr->ctrl.chassis_spin)
            vw = SPIN_VW_SPEED;//小陀螺模式：雷达只下发平移速度，自转角速度取固定值
    }
    else
    {
        chassis_ptr->ctrl.chassis_spin = 0;
        chassis_ptr->ctrl.gimbal_scan = 0;

        vx = 0;
        vy = 0;
        vw = 0;
    }

    // 速度滤波
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[0], vx);
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[1], vy);
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[2], vw);
    const fp32 vx_filter = chassis_ptr->ctrl.chassis_speed_filter[0].out;
    const fp32 vy_filter = chassis_ptr->ctrl.chassis_speed_filter[1].out;
    const fp32 vw_filter = chassis_ptr->ctrl.chassis_speed_filter[2].out;

    // 转换为合速度大小与角度（极坐标格式存储进 given_chassis_v）
    fp32 norm_v;
    arm_sqrt_f32(vx_filter * vx_filter + vy_filter * vy_filter, &norm_v);
    chassis_ptr->ctrl.given_chassis_v[0] = norm_v * LINEAR_TO_RPM;
    chassis_ptr->ctrl.given_chassis_v[1] = atan2f(vy_filter, vx_filter);
    chassis_ptr->ctrl.given_chassis_w = vw_filter * LINEAR_TO_RPM;
}

/**
 * @brief 全向轮底盘运动学解算函数
 * @param vx x向速度 m/s
 * @param vy y向速度 m/s
 * @param vw 自转角速度 rad/s
 * @param spin_yaw 当前Yaw向扭矩偏移
 * @param motor_speeds 输出到3个电机的目标转速
 */
void Omni_Kinematics_Resolve(const fp32 vx, const fp32 vy, const fp32 vw, const fp32 spin_yaw, int16_t* motor_speeds)
{
    const fp32 sin_yaw = sinf(radian_format(spin_yaw));
    const fp32 cos_yaw = cosf(radian_format(spin_yaw));
    const fp32 vx_set = vx * cos_yaw - vy * sin_yaw;
    const fp32 vy_set = vx * sin_yaw + vy * cos_yaw;

    motor_speeds[0] = (int16_t)(-vx_set * 0.667f + vw * 0.333f);
    motor_speeds[1] = (int16_t)(vx_set * 0.333f + vy_set * 0.577f + vw * 0.333f);
    motor_speeds[2] = (int16_t)(vx_set * 0.333f - vy_set * 0.577f + vw * 0.333f);
}

/**
 * @brief 解算3508电机的目标速度（给3个轮毂下发指令速度），以上电时刻向前为y正方向
 */
static void chassis_calc_wheelmotor_speed(robot_t* chassis_ptr)
{
    const fp32 chassis_v = chassis_ptr->ctrl.given_chassis_v[0];
    const fp32 theta = chassis_ptr->ctrl.given_chassis_v[1];
    const fp32 chassis_w = chassis_ptr->ctrl.given_chassis_w;
    const fp32 vx = chassis_v * cosf(theta);
    const fp32 vy = chassis_v * sinf(theta);
    int16_t motor_speeds[3];
#ifdef OMNI
    Omni_Kinematics_Resolve(vx, vy, chassis_w, 0, motor_speeds);
#endif

    chassis_ptr->ctrl.m3508_controller[0].given_speed = motor_speeds[0];
    chassis_ptr->ctrl.m3508_controller[1].given_speed = motor_speeds[1];
    chassis_ptr->ctrl.m3508_controller[2].given_speed = motor_speeds[2];
}

/**
 * @brief PID控制计算
 */
static void chassis_calc_wheelmotor_pidout(robot_t* chassis_ptr)
{
    for (int i = 0; i < 3; i++)
    {
        //PID_calc(&chassis_ptr->ins.pid,,chassis_ptr->ctrl.m3508_controller[i].given_speed);
        PID_calc(&chassis_ptr->ctrl.m3508_controller[i].pid, chassis_ptr->m3508.ecd[i].speed, chassis_ptr->ctrl.m3508_controller[i].given_speed);
        chassis_ptr->ctrl.m3508_controller[i].out = chassis_ptr->ctrl.m3508_controller[i].pid.out[0];
    }
}

/**
 * @brief 将控制计算结果下发到CAN总线上
 */
static void chassis_send_wheelmotor_cmd(const robot_t* chassis_ptr)
{
    int16_t m3508_iq[4];
    for (int i = 0; i < 3; i++)
        m3508_iq[i] = (int16_t)chassis_ptr->ctrl.m3508_controller[i].out;
    m3508_ctrl(&chassis_ptr->m3508, m3508_iq);
}

#if 0 // ===== 以下为排球机构功能（云台/击球/发球），暂时禁用，需要时删掉本行和文末的 #endif =====
static void gimbal_calc_pitch_target(robot_t* chassis_ptr)
{
    if (chassis_ptr->ctrl.shoot_mode == SHOOT_RECEIVE)
        switch (chassis_ptr->ctrl.robot_controller)
        {
            case CHASSIS_RC:
            case CHASSIS_UPC:
                chassis_ptr->ctrl.given_pitch += ((fp32)chassis_ptr->rc.rc_data.ch1 * PITCH_DELTA_MAX);
                break;
            // case CHASSIS_UPC:
            //     chassis_ptr->ctrl.given_pitch = chassis_ptr->nx_ctrl.target_pitch;
            //     break;
            default:
                break;
        }
    if (chassis_ptr->ctrl.given_pitch > PITCH_MAX)
        chassis_ptr->ctrl.given_pitch = PITCH_MAX;
    else if (chassis_ptr->ctrl.given_pitch < PITCH_MIN)
        chassis_ptr->ctrl.given_pitch = PITCH_MIN;
}

static void gimbal_calc_shoot_target(robot_t* chassis_ptr)
{
    if (chassis_ptr->ctrl.robot_controller == CHASSIS_RC)
        chassis_ptr->ctrl.shoot = chassis_ptr->rc.rc_data.roll > 600 ? 1 : 0;
    else if (chassis_ptr->ctrl.robot_controller == CHASSIS_UPC)
    {
        const uint16_t torque = MAX_OF_THREE(chassis.rs02[0].ecd.torch, chassis.rs02[1].ecd.torch, chassis.rs02[2].ecd.torch);
        const uint8_t case2 = (fp32)torque / RS02_UINT16_MAX * 2.0f * RS02_TORCH_MAX - RS02_TORCH_MAX > 2.0f && loop_float_constrain((fp32)chassis.rs02[0].ecd.ecd / RS02_UINT16_MAX * 2.0f * RS02_ECD_MAX - RS02_ECD_MAX, -PI, PI) < 0.10f;
        if (chassis_ptr->nx_ctrl.fire || case2) // case2 || chassis_ptr->vofa->ready ||
            chassis_ptr->ctrl.shoot = 1;
        else
            chassis_ptr->ctrl.shoot = 0;
        if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE) chassis_ptr->ctrl.shoot = 0;
    }
    else
        chassis_ptr->ctrl.shoot = 0;
    if (chassis_ptr->ctrl.shoot && chassis_ptr->ctrl.shoot_step == 0)
        chassis_ptr->ctrl.shoot_step = 1;
}

static void gimbal_shoot_process_1(robot_t* chassis_ptr)
{
    static uint64_t last_time = 0;
    switch (chassis_ptr->ctrl.shoot_step)
    {
        case SHOOT_DEFAULT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.036f;
                chassis_ptr->ctrl.given_hit_speed = 4.0f;
                chassis_ptr->ctrl.given_shoot_angle = SHOOT_FIRE_PREPARE_ANGLE;
                chassis_ptr->ctrl.given_shoot_speed = 2.0f;
                chassis_ptr->ctrl.given_pitch = 0.0f;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                chassis_ptr->ctrl.given_shoot_angle = 0.0f;
                chassis_ptr->ctrl.given_shoot_speed = 2.0f;
            }
            chassis_ptr->ctrl.move = 0;
            break;
        }
        case SHOOT_PREPARE:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.08f;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                last_time = DWT_GetTimeline_us();
                chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_1;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                chassis_ptr->ctrl.shoot_step = SHOOT_HIT;
            }
            break;
        }
        case SHOOT_DELAY_1:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(25)) // 25ms准备时间
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_HIT;
            }
            break;
        }
        case SHOOT_HIT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.75f;
                chassis_ptr->ctrl.given_hit_speed = 32.0f;
                // chassis_ptr->ctrl.given_shoot_angle = 0.2f;
                // chassis_ptr->ctrl.given_shoot_speed = 4.0f;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = 0.75f;
                chassis_ptr->ctrl.given_hit_speed = 32.0f;
                chassis_ptr->ctrl.given_shoot_angle = 0.0f;
                chassis_ptr->ctrl.given_shoot_speed = 4.0f;
            }
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_2;
            break;
        }
        case SHOOT_DELAY_2:
        {
               // chassis_ptr->ctrl.move = 1;
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(80))
            {
                if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
                    chassis_ptr->ctrl.shoot_step = SHOOT_LEANING;
                else
                    chassis_ptr->ctrl.shoot_step = SHOOT_REPARE;
            }
            break;
        }
        case SHOOT_LEANING:
        {
            chassis_ptr->ctrl.given_hit_pos = 0.036f;
            chassis_ptr->ctrl.given_hit_speed = 8.0f;
            chassis_ptr->ctrl.given_pitch = -0.8f;
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_3;
            break;
        }
        case SHOOT_DELAY_3:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(675)) //
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_SHOOT;
            }
            break;
        }
        case SHOOT_SHOOT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_shoot_angle = 2.2f;
                chassis_ptr->ctrl.given_shoot_speed = 22.0f;
            }

            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_4;
            break;
        }
        case SHOOT_DELAY_4:
        {
                chassis_ptr->ctrl.move = 0;
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(500))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_BACK;
            }
            break;
        }
        case SHOOT_BACK:
        {
            chassis_ptr->ctrl.given_shoot_angle = SHOOT_FIRE_PREPARE_ANGLE;
            chassis_ptr->ctrl.given_shoot_speed = 2.0f;
            vesc_speed_ctrl(chassis_ptr->vesc, 0);
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_5;
            break;
        }
        case SHOOT_DELAY_5:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(1500))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_DEFAULT;
            }
            break;
        }
        case SHOOT_REPARE:
        {
            chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
            chassis_ptr->ctrl.given_hit_speed = 8.0f;
            chassis_ptr->ctrl.given_shoot_angle = 0.0f;
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_6;
            break;
        }
        case SHOOT_DELAY_6:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(1250))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_DEFAULT;
            }
            break;
        }
        default:
            break;
    }
}

static void gimbal_shoot_process_2(robot_t* chassis_ptr)
{
    static uint64_t last_time = 0;
    switch (chassis_ptr->ctrl.shoot_step)
    {
        case SHOOT_DEFAULT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.036f;
                chassis_ptr->ctrl.given_hit_speed = 4.0f;
                chassis_ptr->ctrl.given_shoot_angle = SHOOT_FIRE_PREPARE_ANGLE;
                chassis_ptr->ctrl.given_shoot_speed = 2.0f;
                chassis_ptr->ctrl.given_pitch = 0.0f;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                chassis_ptr->ctrl.given_shoot_angle = 0.0f;
                chassis_ptr->ctrl.given_shoot_speed = 2.0f;
            }
            chassis_ptr->ctrl.move = 0;
            break;
        }
        case SHOOT_PREPARE:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.036f;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                chassis_ptr->ctrl.move = 1;
                last_time = DWT_GetTimeline_us();
                chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_1;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                chassis_ptr->ctrl.shoot_step = SHOOT_HIT;
            }
            break;
        }
        case SHOOT_DELAY_1:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(250))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_HIT;
            }
            break;
        }
        case SHOOT_HIT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.50f;
                chassis_ptr->ctrl.given_hit_speed = 32.0f;
                // chassis_ptr->ctrl.given_shoot_angle = 0.2f;
                // chassis_ptr->ctrl.given_shoot_speed = 4.0f;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = 0.70f;
                chassis_ptr->ctrl.given_hit_speed = 32.0f;
                chassis_ptr->ctrl.given_shoot_angle = 0.0f;
                chassis_ptr->ctrl.given_shoot_speed = 4.0f;
            }
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_2;
            break;
        }
        case SHOOT_DELAY_2:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(80))
            {
                if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
                    chassis_ptr->ctrl.shoot_step = SHOOT_LEANING;
                else
                    chassis_ptr->ctrl.shoot_step = SHOOT_REPARE;
            }
            break;
        }
        case SHOOT_LEANING:
        {
            chassis_ptr->ctrl.given_hit_pos = 0.036f;
            chassis_ptr->ctrl.given_hit_speed = 8.0f;
            chassis_ptr->ctrl.given_pitch = -0.8f;
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_3;
            break;
        }
        case SHOOT_DELAY_3:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(240)) //
            {
                chassis_ptr->ctrl.move = 0;
            }
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(480)) //
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_SHOOT;
            }
            break;
        }
        case SHOOT_SHOOT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_shoot_angle = -2.2f;
                chassis_ptr->ctrl.given_shoot_speed = 44.0f;
            }

            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_4;
            break;
        }
        case SHOOT_DELAY_4:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(300))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_BACK;
            }
            break;
        }
        case SHOOT_BACK:
        {
            chassis_ptr->ctrl.given_shoot_angle = SHOOT_FIRE_PREPARE_ANGLE;
            chassis_ptr->ctrl.given_shoot_speed = 2.0f;
            vesc_speed_ctrl(chassis_ptr->vesc, 0);
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_5;
            break;
        }
        case SHOOT_DELAY_5:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(3000))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_DEFAULT;
            }
            break;
        }
        case SHOOT_REPARE:
        {
            chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
            chassis_ptr->ctrl.given_hit_speed = 8.0f;
            chassis_ptr->ctrl.given_shoot_angle = 0.0f;
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_6;
            break;
        }
        case SHOOT_DELAY_6:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(1250))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_DEFAULT;
            }
            break;
        }
        default:
            break;
    }
}

static void gimbal_shoot_process_3(robot_t* chassis_ptr)
{
    static uint64_t last_time = 0;
    switch (chassis_ptr->ctrl.shoot_step)
    {
        case SHOOT_DEFAULT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.036f;
                chassis_ptr->ctrl.given_hit_speed = 4.0f;
                chassis_ptr->ctrl.given_shoot_angle = 0.0f;
                chassis_ptr->ctrl.given_shoot_speed = 0.0f;
                chassis_ptr->ctrl.given_pitch = -0.05f;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                chassis_ptr->ctrl.given_shoot_angle = 0.0f;
                chassis_ptr->ctrl.given_shoot_speed = 2.0f;
            }
            break;
        }
        case SHOOT_PREPARE:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.08f;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                last_time = DWT_GetTimeline_us();
                chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_1;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                chassis_ptr->ctrl.shoot_step = SHOOT_HIT;
            }
            break;
        }
        case SHOOT_DELAY_1:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(25)) // 25ms准备时间
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_HIT;
            }
            break;
        }
        case SHOOT_HIT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.75f;
                chassis_ptr->ctrl.given_hit_speed = 22.0f;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = 0.75f;
                chassis_ptr->ctrl.given_hit_speed = 32.0f;
            }
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_2;
            break;
        }
        case SHOOT_DELAY_2:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(645))
            {
                if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
                    chassis_ptr->ctrl.shoot_step = SHOOT_LEANING;
                else
                    chassis_ptr->ctrl.shoot_step = SHOOT_REPARE;
            }
            break;
        }
        case SHOOT_LEANING:
        {
            chassis_ptr->ctrl.given_hit_pos = 0.06f;
            chassis_ptr->ctrl.given_hit_speed = 8.0f;
            chassis_ptr->ctrl.given_pitch = -0.6f;
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_3;
            break;
        }
        case SHOOT_DELAY_3:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(310)) //
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_SHOOT;
            }
            break;
        }
        case SHOOT_SHOOT:
        {
            chassis_ptr->ctrl.given_hit_pos = 0.75f;
            chassis_ptr->ctrl.given_hit_speed = 32.0f;
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_4;
            break;
        }
        case SHOOT_DELAY_4:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(50))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_BACK;
            }
            break;
        }
        case SHOOT_BACK:
        {
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_5;
            break;
        }
        case SHOOT_DELAY_5:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(50))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_DEFAULT;
            }
            break;
        }
        case SHOOT_REPARE:
        {
            chassis_ptr->ctrl.given_hit_pos = SHOOT_RECEIVE_POS;
            chassis_ptr->ctrl.given_hit_speed = 8.0f;
            chassis_ptr->ctrl.given_shoot_angle = 0.0f;
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_6;
            break;
        }
        case SHOOT_DELAY_6:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(1250))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_DEFAULT;
            }
            break;
        }
        default:
            break;
    }
}

void rs02_send_cmd_task(const void* argument)
{
    const robot_t* chassis_ptr = (robot_t*)argument;
    static uint8_t reconnect = 0;
    while(1)
    {
        uint8_t rs02_error = 0;
        for (uint8_t i = 0; i < 5; i++)
            rs02_error |= chassis_ptr->rs02[i].ecd.error;
        if (chassis_ptr->ctrl.rc_offline || chassis_ptr->ctrl.robot_controller == CHASSIS_SHUTDOWN || chassis_ptr->ctrl.robot_controller == CHASSIS_RC_OFFLINE || rs02_error)
        {
            for (uint8_t i = 0; i < 5; i++)
            {
                rs02_disable_private(&chassis_ptr->rs02[i], 1);
                osDelay(1);
            }
            reconnect = 1;
            osDelay(1000);
        }
        else if (!chassis_ptr->ctrl.rc_offline && chassis_ptr->ctrl.robot_controller != CHASSIS_SHUTDOWN && chassis_ptr->ctrl.robot_controller != CHASSIS_RC_OFFLINE && !rs02_error && reconnect)
        {
            for (uint8_t i = 0; i < 5; i++)
            {
                rs02_enable_private(&chassis_ptr->rs02[i]);
                osDelay(1);
            }
            reconnect = 0;
        }
        if (!chassis_ptr->ctrl.rc_offline && chassis_ptr->ctrl.robot_controller != CHASSIS_SHUTDOWN && chassis_ptr->ctrl.robot_controller != CHASSIS_RC_OFFLINE && !rs02_error)
        {
            rs02_ctrl_3motor_pos_private(&chassis_ptr->rs02[0], &chassis_ptr->rs02[1], &chassis_ptr->rs02[2], chassis_ptr->ctrl.given_hit_pos, chassis_ptr->ctrl.given_hit_speed);
            rs02_ctrl_pos_private(&chassis_ptr->rs02[3], chassis_ptr->ctrl.given_pitch, 4.0f);
            osDelay(1);
            rs02_ctrl_pos_private(&chassis_ptr->rs02[4], chassis_ptr->ctrl.given_shoot_angle + ((fp32)chassis_ptr->rs02[4].ecd.ecd_offset / RS02_UINT16_MAX * 2.0f * RS02_ECD_MAX - RS02_ECD_MAX), chassis_ptr->ctrl.given_shoot_speed);
            //rs02_ctrl_pos_private(&chassis_ptr->rs02[4], 0.0f + ((fp32)chassis_ptr->rs02[4].ecd.ecd_offset / RS02_UINT16_MAX * 2.0f * RS02_ECD_MAX - RS02_ECD_MAX), chassis_ptr->ctrl.given_shoot_speed);
        }
        osDelay(1);

    }
}

/**
 * @brief 将底盘位置信息发送到上位机
 * pitch轴数据可能需要改为计算电机编码器获得，是否要改取决于C板是否能安装在云台上
 */
static void chasssis_send_pos(const robot_t* chassis_ptr)
{
    const fp32 motor_pitch = (fp32)chassis_ptr->rs02[3].ecd.ecd / RS02_UINT16_MAX * 2.0f * RS02_ECD_MAX - RS02_ECD_MAX;
    NX_SendPos(&chassis_ptr->nx_ctrl, chassis_ptr->odom.x, chassis_ptr->odom.y, chassis_ptr->odom.theta, motor_pitch);
}
static void chasssis_send_vec(const robot_t* chassis_ptr)
{
    NX_SendVec(&chassis_ptr->nx_ctrl, chassis_ptr->odom.vx, chassis_ptr->odom.vy);
}
static void chasssis_send_motorspeed(const robot_t* chassis_ptr)
{
    int16_t speed[3] = {chassis_ptr->m3508.ecd[0].speed, chassis_ptr->m3508.ecd[1].speed, chassis_ptr->m3508.ecd[2].speed};
    odom_send_speed(speed);
}
#endif // ===== 排球机构功能结束 =====