#ifndef NEW_CHASSIS_RLS_H
#define NEW_CHASSIS_RLS_H

#ifndef RLS_H
#define RLS_H

#include "typedef.h"

// 矩阵结构体定义
typedef struct {
    float data[2][2];  // 2x2矩阵
} Matrix2x2;

typedef struct {
    float data[2];      // 2x1向量
} Vector2;

// RLS算法结构体
typedef struct {
    // 算法参数
    float lambda;           // 遗忘因子 (0 < λ ≤ 1)
    float delta;           // 协方差矩阵初始值

    // 状态变量
    Matrix2x2 transMatrix;  // 协方差矩阵 P(k)
    Vector2 gainVector;     // 增益向量 K(k)
    Vector2 paramsVector;   // 参数向量 θ(k)
    Vector2 defaultParams;  // 默认参数向量

    // 统计信息
    uint32_t updateCnt;     // 更新次数
    uint32_t lastUpdateTick; // 最后更新时间戳

    // 输出
    float output;           // 滤波/估计输出
} RLS_Handle;

/**
 * @brief 初始化RLS算法
 * @param handle RLS句柄指针
 * @param delta 协方差矩阵初始值 (通常设为小的正数，如1e-5)
 * @param lambda 遗忘因子 (0 < λ ≤ 1, 通常0.95~0.99)
 * @param initParams 初始参数向量，为NULL时使用零向量
 */
void RLS_Init(RLS_Handle* handle, float delta, float lambda, const float* initParams);

/**
 * @brief 重置RLS算法
 * @param handle RLS句柄指针
 */
void RLS_Reset(RLS_Handle* handle);

/**
 * @brief 执行一次RLS更新
 * @param handle RLS句柄指针
 * @param samples 输入样本向量 [x1(k), x2(k)]^T
 * @param actualOutput 实际输出 y(k)
 * @return 更新后的参数向量指针
 */
const Vector2* RLS_Update(RLS_Handle* handle, const Vector2* samples, float actualOutput);

/**
 * @brief 设置参数向量
 * @param handle RLS句柄指针
 * @param params 新的参数向量
 */
void RLS_SetParamVector(RLS_Handle* handle, const Vector2* params);

/**
 * @brief 获取当前参数向量
 * @param handle RLS句柄指针
 * @return 参数向量指针
 */
const Vector2* RLS_GetParamsVector(const RLS_Handle* handle);

/**
 * @brief 获取RLS估计输出
 * @param handle RLS句柄指针
 * @return 估计输出值
 */
float RLS_GetOutput(const RLS_Handle* handle);

/**
 * @brief 获取更新次数
 * @param handle RLS句柄指针
 * @return 更新次数
 */
uint32_t RLS_GetUpdateCount(const RLS_Handle* handle);

/**
 * @brief 获取最后更新时间戳
 * @param handle RLS句柄指针
 * @return 时间戳
 */
uint32_t RLS_GetLastUpdateTick(const RLS_Handle* handle);

/**
 * @brief 验证RLS参数是否有效
 * @param handle RLS句柄指针
 * @return true: 有效, false: 无效
 */
uint8_t RLS_Validate(const RLS_Handle* handle);

#endif // RLS_H

#endif //NEW_CHASSIS_RLS_H