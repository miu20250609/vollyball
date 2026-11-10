/**
 * @file user_lib.c
 * @brief 数学函数模块
 * 常见数学函数。
 * @version 1.0
 * @date 2026-01-17
 */

#include "string.h"
#include "user_lib.h"
#include "math.h"

/*---------------------------------------数据处理--------------------------------------- */
/**
  * @brief 将4位uint8_t转换为1个float
  * @param data 4个uint8_t
  * @param f1 1个float
  */
void unpack_4bytes_to_floats(const uint8_t data[4], float* f1) {
    uint32_t u1;
    
    // 小端序
    u1 = (uint32_t)data[0] | 
         ((uint32_t)data[1] << 8) | 
         ((uint32_t)data[2] << 16) | 
         ((uint32_t)data[3] << 24);
    
    memcpy(f1, &u1, sizeof(float));
}

/**
 * @brief 将1个float转换为4位uint8_t
 * @param f1 1个float
 * @param data 4个uint8_t
 */
void pack_float_to_4bytes(float f1, uint8_t data[4]) {

    uint32_t u1 = *((uint32_t*)&f1);
    
    data[3] = (uint8_t)((u1 >> 24) & 0xFF); 
    data[2] = (uint8_t)((u1 >> 16) & 0xFF);
    data[1] = (uint8_t)((u1 >> 8)  & 0xFF);
    data[0] = (uint8_t)(u1 & 0xFF);    
}
/**
 * @brief 查找缓冲区中所有帧头的位置
 *
 * @param buf 输入缓冲区
 * @param buf_len 缓冲区长度
 * @param header 要查找的帧头
 * @param header_len 帧头长度
 * @param positions 存储找到的位置的数组
 * @param max_positions 最大可存储位置数量
 * @return int 实际找到的帧头数量
 */
uint16_t find_frame_headers(
    const uint8_t *buf,
    const uint16_t buf_len,
    const uint8_t *header,
    const uint8_t header_len,
    uint16_t *positions,
    const uint16_t max_positions
) {
    int found_count = 0;

    if (header_len == 0 || buf_len < header_len || max_positions == 0) {
        return 0;
    }

    for (uint16_t i = 0; i <= buf_len - header_len; i++) {
        uint8_t match = 1;
        for (uint8_t j = 0; j < header_len; j++) {
            if (buf[i + j] != header[j]) {
                match = 0;
                break;
            }
        }

        if (match) {
            if (found_count < max_positions) {
                positions[found_count] = i;
                found_count++;
            } else {
                break;
            }
        }
    }

    return found_count;
}

/**
 * @brief 更新各种角度信息
 *
 * @param angle 角度信息指针
 * @param new_angle_deg yaw、pitch、roll角度值
 */
void Angle_Update(angle_t *angle, const float new_angle_deg[3])
{
    angle->yaw_deg = new_angle_deg[0];
    angle->pitch_deg = new_angle_deg[1];
    angle->roll_deg = new_angle_deg[2];

    angle->yaw_rad = angle->yaw_deg / 57.295779513f;
    angle->pitch_rad = angle->pitch_deg / 57.295779513f;
    angle->roll_rad = angle->roll_deg / 57.295779513f;

    if (angle->yaw_deg - angle->yaw_angle_last > 180.0f)
        angle->yaw_round_count--;
    else if (angle->yaw_deg - angle->yaw_angle_last < -180.0f)
        angle->yaw_round_count++;
    angle->yaw_total_angle = 360.0f * (fp32)angle->yaw_round_count + angle->yaw_deg;
    angle->yaw_angle_last = angle->yaw_deg;
}
/*------------------------------------------------------------------------------------- */

/*-------------------------------------常用数学函数------------------------------------- */
// 快速开方
float q_sqrt(float x)
{
    float delta;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    float y = x / 2;

    // refine
    const float maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 快速平方倒数
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86- (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}
    
// 判符函数 
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//float死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int16死区
int16_t int16_deadband(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//float限幅
void float_constrain(float* Value, float minValue, float maxValue)
{
    if (*Value < minValue)
        *Value = minValue;
    else if (*Value > maxValue)
        *Value = maxValue;
}

//int16限幅
void int16_constrain(int16_t* Value, int16_t minValue, int16_t maxValue)
{
    if (*Value < minValue)
        *Value = minValue;
    else if (*Value > maxValue)
        *Value = maxValue;
}

//float循环限幅
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//int循环限幅
int loop_int_constrain(int Input, const int minValue, const int maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        const int len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        const int len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度归一 -PI~PI
float radian_format(const float Rad)
{
		return loop_float_constrain(Rad, -PI, PI);
}
//角度归一 -180~180
float theta_format(const float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(const float raw)
{
    int integer = (int)raw;
    const float decimal = raw - (fp32)integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}
/*------------------------------------------------------------------------------------- */

/*---------------------------------------一阶滤波--------------------------------------- */
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param first_order_filter_type 一阶低通滤波结构体
  * @param frame_period 间隔的时间，单位 s
  * @param num 滤波参数
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief 一阶低通滤波计算
  * @author RM
  * @param first_order_filter_type 一阶低通滤波结构体
  * @param input 间隔的时间，单位 s
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
}
/*------------------------------------------------------------------------------------- */

/*---------------------------------------斜波函数--------------------------------------- */
/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      ramp_source_type  斜波函数结构体指针
  * @param[in]      frame_period      帧周期，单位 s
  * @param[in]      max               最大值
  * @param[in]      min               最小值
  * @retval         空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，输入值在最小值和最大值之间变化，输出值在最小值和最大值之间变化
  * @author         RM
  * @param[in]      ramp_source_type  斜波函数结构体指针
  * @param[in]      input             输入值
  * @retval         输出值
  */
float ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
    return ramp_source_type->out;
}
/*------------------------------------------------------------------------------------- */

/*--------------------------------------最小二乘法-------------------------------------- */
/**
  * @brief 获取最小二乘法的初始化
  * @param OLS 最小二乘法结构体
  * @param order 阶数
  */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
  * @brief          获取最小二乘法的导数
  * @param OLS 最小二乘法结构体
  * @param deltax 输入时间增量
  * @param y 输入值
  */
void OLS_Update(Ordinary_Least_Squares_t *OLS, const float deltax, const float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * (fp32)OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * (fp32)OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * (fp32)OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= (fp32)OLS->Order;
}

/**
  * @brief          获取最小二乘法的导数
  * @param OLS 最小二乘法结构体
  * @param deltax 输入时间增量
  * @param y 输入值
  * @retval 斜率k
  */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, const float deltax, const float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * (float)OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * (float)OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= (float)OLS->Order;

    return OLS->k;
}

/**
  * @brief 获取最小二乘法的导数
  * @param OLS  最小二乘法结构体
  * @retval 斜率k
  */
float Get_OLS_Derivative(const Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
  * @brief 获取最小二乘法平滑后的值
  * @param OLS 最小二乘法结构体
  * @param deltax 输入时间增量
  * @param y 输入值
  * @retval 平滑后的值
  */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * (fp32)OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * (fp32)OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * (fp32)OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= (fp32)OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
  * @brief 获取最小二乘法平滑后的值
  * @param OLS 最小二乘法结构体
  * @retval 平滑后的值
  */
float Get_OLS_Smooth(const Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

// 四元数转欧拉角 (角度制)
void quaternion_to_euler(const fp32 q[4], fp32 euler_angle[3])
{
    if (q == NULL || euler_angle == NULL) return;

    // yaw (z-axis rotation)
    const fp32 siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    const fp32 cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    euler_angle[0] = atan2f(siny_cosp, cosy_cosp) * RADIAN_COEF;

    // pitch (y-axis rotation)
    const fp32 sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (fabs(sinp) >= 1)
        euler_angle[1] = copysignf(90.0f, sinp); // use 90 degrees if out of range
    else
        euler_angle[1] = asinf(sinp) * RADIAN_COEF;

    // roll (x-axis rotation)
    const fp32 sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    const fp32 cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    euler_angle[2] = atan2f(sinr_cosp, cosr_cosp) * RADIAN_COEF;
}

// 欧拉角转四元数 (角度制)
void euler_to_quaternion(const fp32 euler_angle[3], fp32 q[4])
{
    if (euler_angle == NULL || q == NULL) return;

    const fp32 yaw_rad = euler_angle[0] / RADIAN_COEF;
    const fp32 pitch_rad = euler_angle[1] / RADIAN_COEF;
    const fp32 roll_rad = euler_angle[2] / RADIAN_COEF;

    const fp32 cy = cosf(yaw_rad * 0.5f);
    const fp32 sy = sinf(yaw_rad * 0.5f);
    const fp32 cp = cosf(pitch_rad * 0.5f);
    const fp32 sp = sinf(pitch_rad * 0.5f);
    const fp32 cr = cosf(roll_rad * 0.5f);
    const fp32 sr = sinf(roll_rad * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}
