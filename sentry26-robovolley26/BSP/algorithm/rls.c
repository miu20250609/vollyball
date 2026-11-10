#include "rls.h"
#include <string.h>
#include "math.h"

// 内部辅助函数声明
static float Vector2_Dot(const Vector2* a, const Vector2* b);
static void Matrix2x2_MultiplyVector(const Matrix2x2* mat, const Vector2* vec, Vector2* result);
static void Matrix2x2_OuterProduct(const Vector2* a, const Vector2* b, Matrix2x2* result);
static void Matrix2x2_MultiplyScalar(Matrix2x2* mat, float scalar);
static void Matrix2x2_Subtract(Matrix2x2* a, const Matrix2x2* b);
//static void Matrix2x2_Add(Matrix2x2* a, const Matrix2x2* b);

// 向量点积
static float Vector2_Dot(const Vector2* a, const Vector2* b) {
    return a->data[0] * b->data[0] + a->data[1] * b->data[1];
}

// 矩阵乘以向量: result = mat * vec
static void Matrix2x2_MultiplyVector(const Matrix2x2* mat, const Vector2* vec, Vector2* result) {
    result->data[0] = mat->data[0][0] * vec->data[0] + mat->data[0][1] * vec->data[1];
    result->data[1] = mat->data[1][0] * vec->data[0] + mat->data[1][1] * vec->data[1];
}

// 外积: result = a * b^T
static void Matrix2x2_OuterProduct(const Vector2* a, const Vector2* b, Matrix2x2* result) {
    result->data[0][0] = a->data[0] * b->data[0];
    result->data[0][1] = a->data[0] * b->data[1];
    result->data[1][0] = a->data[1] * b->data[0];
    result->data[1][1] = a->data[1] * b->data[1];
}

// 矩阵乘以标量
static void Matrix2x2_MultiplyScalar(Matrix2x2* mat, float scalar) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            mat->data[i][j] *= scalar;
        }
    }
}

// 矩阵减法: a = a - b
static void Matrix2x2_Subtract(Matrix2x2* a, const Matrix2x2* b) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            a->data[i][j] -= b->data[i][j];
        }
    }
}

// // 矩阵加法: a = a + b
// static void Matrix2x2_Add(Matrix2x2* a, const Matrix2x2* b) {
//     for (int i = 0; i < 2; i++) {
//         for (int j = 0; j < 2; j++) {
//             a->data[i][j] += b->data[i][j];
//         }
//     }
// }

void RLS_Init(RLS_Handle* handle, float delta, float lambda, const float* initParams) {
    if(handle == NULL)
        return;
    if(lambda < 0.0f && lambda > 1.0f)
        return;
    if(delta <= 0.0f)
        return;

    // 初始化参数
    handle->lambda = lambda;
    handle->delta = delta;
    handle->updateCnt = 0;
    handle->lastUpdateTick = 0;
    handle->output = 0.0f;

    // 初始化协方差矩阵为单位矩阵乘以delta
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (i == j) {
                handle->transMatrix.data[i][j] = delta;
            } else {
                handle->transMatrix.data[i][j] = 0.0f;
            }
        }
    }

    // 初始化为零向量
    handle->gainVector.data[0] = 0.0f;
    handle->gainVector.data[1] = 0.0f;

    handle->paramsVector.data[0] = 0.0f;
    handle->paramsVector.data[1] = 0.0f;

    // 如果有初始参数，使用初始参数
    if (initParams != NULL) {
        handle->paramsVector.data[0] = initParams[0];
        handle->paramsVector.data[1] = initParams[1];
        handle->defaultParams.data[0] = initParams[0];
        handle->defaultParams.data[1] = initParams[1];
    } else {
        handle->defaultParams.data[0] = 0.0f;
        handle->defaultParams.data[1] = 0.0f;
    }
}

void RLS_Reset(RLS_Handle* handle) {
    if(handle == NULL)
        return;

    // 重置协方差矩阵
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (i == j) {
                handle->transMatrix.data[i][j] = handle->delta;
            } else {
                handle->transMatrix.data[i][j] = 0.0f;
            }
        }
    }

    // 重置增益向量
    handle->gainVector.data[0] = 0.0f;
    handle->gainVector.data[1] = 0.0f;

    // 重置参数向量
    handle->paramsVector.data[0] = 0.0f;
    handle->paramsVector.data[1] = 0.0f;

    // 重置统计信息
    handle->updateCnt = 0;
    handle->lastUpdateTick = 0;
    handle->output = 0.0f;
}

const Vector2* RLS_Update(RLS_Handle* handle, const Vector2* samples, const float actualOutput) {

    // 步骤1: 计算中间变量 φ^T * P(k-1) * φ
    Vector2 tempVector;
    Matrix2x2_MultiplyVector(&handle->transMatrix, samples, &tempVector);
    float denominator = handle->lambda + Vector2_Dot(samples, &tempVector);

    // 避免除以零
    if (fabsf(denominator) < 1e-10f) {
        denominator = 1e-10f;
    }

    // 步骤2: 计算增益向量 K(k) = P(k-1) * φ / (λ + φ^T * P(k-1) * φ)
    handle->gainVector.data[0] = tempVector.data[0] / denominator;
    handle->gainVector.data[1] = tempVector.data[1] / denominator;

    // 步骤3: 计算估计误差 e(k) = y(k) - φ^T * θ(k-1)
    const float phi_theta = Vector2_Dot(samples, &handle->paramsVector);
    const float error = actualOutput - phi_theta;

    // 步骤4: 更新参数向量 θ(k) = θ(k-1) + K(k) * e(k)
    handle->paramsVector.data[0] += handle->gainVector.data[0] * error;
    handle->paramsVector.data[1] += handle->gainVector.data[1] * error;

    // 步骤5: 更新协方差矩阵 P(k) = (P(k-1) - K(k) * φ^T * P(k-1)) / λ
    // 计算 K(k) * φ^T
    Matrix2x2 K_phiT;
    Matrix2x2_OuterProduct(&handle->gainVector, samples, &K_phiT);

    // 计算 K(k) * φ^T * P(k-1)
    Matrix2x2 K_phiT_P;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            K_phiT_P.data[i][j] = 0.0f;
            for (int k = 0; k < 2; k++) {
                K_phiT_P.data[i][j] += K_phiT.data[i][k] * handle->transMatrix.data[k][j];
            }
        }
    }

    // 计算 P(k) = (P(k-1) - K(k) * φ^T * P(k-1)) / λ
    Matrix2x2_Subtract(&handle->transMatrix, &K_phiT_P);
    Matrix2x2_MultiplyScalar(&handle->transMatrix, 1.0f / handle->lambda);

    // 更新统计信息
    handle->updateCnt++;

    // 计算并存储输出
    handle->output = phi_theta;  // 更新前的估计值

    return &handle->paramsVector;
}

void RLS_SetParamVector(RLS_Handle* handle, const Vector2* params) {
    if(handle == NULL || params == NULL)
        return;

    handle->paramsVector.data[0] = params->data[0];
    handle->paramsVector.data[1] = params->data[1];

    handle->defaultParams.data[0] = params->data[0];
    handle->defaultParams.data[1] = params->data[1];
}

const Vector2* RLS_GetParamsVector(const RLS_Handle* handle) {
    return &handle->paramsVector;
}

float RLS_GetOutput(const RLS_Handle* handle) {
    return handle->output;
}

uint32_t RLS_GetUpdateCount(const RLS_Handle* handle) {
    return handle->updateCnt;
}

uint32_t RLS_GetLastUpdateTick(const RLS_Handle* handle) {
    return handle->lastUpdateTick;
}

uint8_t RLS_Validate(const RLS_Handle* handle) {
    if (handle == NULL) {
        return 0;
    }

    if (handle->lambda <= 0.0f || handle->lambda > 1.0f) {
        return 0;
    }

    if (handle->delta <= 0.0f) {
        return 0;
    }

    return 1;
}