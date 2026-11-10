#ifndef USER_LIB_H
#define USER_LIB_H
#include "typedef.h"
#include "cmsis_os.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* math relevant */
/* radian coefficient */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct
{
    float roll_deg, yaw_deg, pitch_deg;
    float roll_rad, yaw_rad, pitch_rad;
    float yaw_total_angle;
    int16_t yaw_round_count;
    float yaw_angle_last;
} angle_t;

typedef struct
{
    float input;        //输入值
    float out;          //输出值
    float min_value;    //输出最小值
    float max_value;    //输出最大值
    float frame_period; //时间周期
} ramp_function_source_t;

typedef struct
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;

typedef struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num;       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

float q_sqrt(float x);

void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

float abs_limit(float num, float Limit);
float sign(float value);
float float_deadband(float Value, float minValue, float maxValue);
int16_t int16_deadband(int16_t Value, int16_t minValue, int16_t maxValue);
void float_constrain(float* Value, float minValue, float maxValue);
void int16_constrain(int16_t* Value, int16_t minValue, int16_t maxValue);
float loop_float_constrain(float Input, float minValue, float maxValue);
int loop_int_constrain(int Input, int minValue, int maxValue);
float radian_format(float Rad);
float theta_format(float Ang);
int float_rounding(float raw);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(const Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(const Ordinary_Least_Squares_t *OLS);

void unpack_4bytes_to_floats(const uint8_t data[4], float* f1);
void pack_float_to_4bytes(float f1, uint8_t data[4]);
void Angle_Update(angle_t *angle, const float new_angle_deg[3]);
uint16_t find_frame_headers(
    const uint8_t *buf,
    uint16_t buf_len,
    const uint8_t *header,
    uint8_t header_len,
    uint16_t *positions,
    uint16_t max_positions
);

// 四元数转欧拉角 (角度制)
void quaternion_to_euler(const fp32 q[4], fp32 euler_angle[3]);

// 欧拉角转四元数 (角度制)
void euler_to_quaternion(const fp32 euler_angle[3], fp32 q[4]);

#endif
