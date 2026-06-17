#include "quaternion.h"
#include <math.h>                           

float RtA = 57.2957795f; // 弧度转角度系数
// IMU660RA 陀螺仪量程通常为 +-2000dps, 对应 16位 ADC (65536)
// 弧度每秒系数：(4000 / 65536) * (PI / 180)
float Gyro_Gr = 0.001127f ;//0.00106526f;
#define PITCH_S_CORR  1.049f
#define YAW_S_CORR    1.000f  
#define ROLL_S_CORR   1.000f

#define squa(Sq) (((float)Sq) * ((float)Sq))

static float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;
    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y));
    return y;
}

static float normAccz; 

//void Common_IMU_GetEulerAngle(Gyro_Accel_Struct *gyroAccel, Euler_struct *eulerAngle, float dt)
//{
//    // 定义中间变量结构体
//    struct V { float x; float y; float z; } Gravity, Acc, Gyro, AccGravity;
//    static struct V GyroIntegError = {0};
//    static float KpDef = 0.2f;      // 比例增益限制加速度计补偿速度
//    static float KiDef = 0.0003f;   // 积分增益消除漂移
//    static Quaternion_Struct NumQ = {1, 0, 0, 0};
//    
//    float q0_t, q1_t, q2_t, q3_t;
//    float NormQuat;
//    float HalfTime = dt * 0.5f;
//
//    // 1. 提取等效旋转矩阵中的重力分量 (从四元数推导重力在机体坐标系下的方向)
//    Gravity.x = 2.0f * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
//    Gravity.y = 2.0f * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
//    Gravity.z = 1.0f - 2.0f * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
//
//    // 2. 加速度归一化 (使用匹配你 mpu660ra.c 的变量名)
//    NormQuat = Q_rsqrt(squa(gyroAccel->accel.imu660ra_acc_x) + squa(gyroAccel->accel.imu660ra_acc_y) + squa(gyroAccel->accel.imu660ra_acc_z));
//    
//    Acc.x = (float)gyroAccel->accel.imu660ra_acc_x * NormQuat;
//    Acc.y = (float)gyroAccel->accel.imu660ra_acc_y * NormQuat;
//    Acc.z = (float)gyroAccel->accel.imu660ra_acc_z * NormQuat;
//
//    // 3. 叉乘误差：测量值(Acc)与参考值(Gravity)的偏差
//    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
//    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
//    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
//
//    // 4. 误差积分补偿
//    GyroIntegError.x += AccGravity.x * KiDef;
//    GyroIntegError.y += AccGravity.y * KiDef;
//    GyroIntegError.z += AccGravity.z * KiDef;
//
//    // 5. 修正后的角速度 (融合加速度计补偿)
////    Gyro.x = (float)gyroAccel->gyro.imu660ra_gyro_x * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;
////    Gyro.y = (float)gyroAccel->gyro.imu660ra_gyro_y * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
////    Gyro.z = (float)gyroAccel->gyro.imu660ra_gyro_z * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
//    Gyro.x = (float)gyroAccel->gyro.imu660ra_gyro_x * Gyro_Gr * ROLL_S_CORR + KpDef * AccGravity.x + GyroIntegError.x;
//    Gyro.y = (float)gyroAccel->gyro.imu660ra_gyro_y * Gyro_Gr * PITCH_S_CORR + KpDef * AccGravity.y + GyroIntegError.y;
//    Gyro.z = (float)gyroAccel->gyro.imu660ra_gyro_z * Gyro_Gr * YAW_S_CORR;
//
//    // 6. 一阶龙格库塔更新四元数
//    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
//    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
//    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
//    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;
//
//    NumQ.q0 += q0_t; NumQ.q1 += q1_t; NumQ.q2 += q2_t; NumQ.q3 += q3_t;
//
//    // 7. 四元数归一化
//    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
//    NumQ.q0 *= NormQuat; NumQ.q1 *= NormQuat; NumQ.q2 *= NormQuat; NumQ.q3 *= NormQuat;
//
//    // 8. 转换成欧拉角
//    // 俯仰角 (Pitch): vecxZ
//    float vecxZ = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
//    // 横滚角 (Roll): vecyZ / veczZ
//    float vecyZ = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
//    float veczZ = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
//
//    eulerAngle->pitch = -asin(vecxZ) * RtA; 
//    eulerAngle->roll  = atan2f(vecyZ, veczZ) * RtA;
// 
//    // 偏航角 (Yaw): 使用陀螺仪直接积分 (四元数Yaw在没有磁力计时会漂移)
//    float yaw_rate = (float)gyroAccel->gyro.imu660ra_gyro_z * (4000.0f / 65536.0f);
//    if(fabs(yaw_rate) > 0.5f) {
//        eulerAngle->yaw += yaw_rate * dt;
//    }
//
//    // 记录Z轴净加速度
//    normAccz = gyroAccel->accel.imu660ra_acc_x * vecxZ + 
//               gyroAccel->accel.imu660ra_acc_y * vecyZ + 
//               gyroAccel->accel.imu660ra_acc_z * veczZ;
//}

void Common_IMU_GetEulerAngle(Gyro_Accel_Struct *gyroAccel, Euler_struct *eulerAngle, float dt)
{
    // 1. 定义中间变量结构体
    struct V { float x; float y; float z; } Gravity, Acc, Gyro, AccGravity;
    static struct V GyroIntegError = {0};
    
    // 增益参数微调：Kp 决定加速度计纠偏力度，Ki 决定消除漂移的速度
    static float KpDef = 0.12f;      // 建议从 0.2 稍微调小，减少高角度时的干扰
    static float KiDef = 0.0001f;   // 建议从 0.0003 稍微调小
    static Quaternion_Struct NumQ = {1, 0, 0, 0};
    
    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // 2. 提取等效旋转矩阵中的重力分量 (从四元数推导重力在机体坐标系下的方向)
    // 这实际上是旋转矩阵的第三列：[vx, vy, vz]
    Gravity.x = 2.0f * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2.0f * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1.0f - 2.0f * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // 3. 加速度归一化
    float acc_norm_sq = squa(gyroAccel->accel.imu660ra_acc_x) + 
                        squa(gyroAccel->accel.imu660ra_acc_y) + 
                        squa(gyroAccel->accel.imu660ra_acc_z);
    
    // 增加安全性检查：防止除以0导致的算法崩溃
    if (acc_norm_sq > 0.0f) {
        NormQuat = Q_rsqrt(acc_norm_sq);
        //Acc.x = (float)gyroAccel->accel.imu660ra_acc_x * NormQuat;
        Acc.x = -(float)gyroAccel->accel.imu660ra_acc_x * NormQuat;
        //Acc.y = (float)gyroAccel->accel.imu660ra_acc_y * NormQuat;
        Acc.y = -(float)gyroAccel->accel.imu660ra_acc_y * NormQuat;
        Acc.z = (float)gyroAccel->accel.imu660ra_acc_z * NormQuat;

        // 4. 叉乘误差：测量值(Acc)与参考值(Gravity)的偏差
        AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
        AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
        AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

        // 5. 误差积分补偿
        GyroIntegError.x += AccGravity.x * KiDef;
        GyroIntegError.y += AccGravity.y * KiDef;
        GyroIntegError.z += AccGravity.z * KiDef;
    } else {
        AccGravity.x = AccGravity.y = AccGravity.z = 0.0f;
    }

    // 6. 修正后的角速度 (融合分轴补偿和加速度计修正)
    // 使用你定义的分轴 CORR 系数
    Gyro.x = -(float)gyroAccel->gyro.imu660ra_gyro_x * Gyro_Gr * ROLL_S_CORR  + KpDef * AccGravity.x + GyroIntegError.x;
    Gyro.y = -(float)gyroAccel->gyro.imu660ra_gyro_y * Gyro_Gr * PITCH_S_CORR + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = (float)gyroAccel->gyro.imu660ra_gyro_z * Gyro_Gr * YAW_S_CORR   + KpDef * AccGravity.z + GyroIntegError.z;

    // 7. 一阶龙格库塔更新四元数
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t; NumQ.q1 += q1_t; NumQ.q2 += q2_t; NumQ.q3 += q3_t;

    // 8. 四元数归一化
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat; NumQ.q1 *= NormQuat; NumQ.q2 *= NormQuat; NumQ.q3 *= NormQuat;

    // 9. 最终转换成欧拉角 (使用 atan2 彻底解决 90 度和 Roll 跳变问题)
    // 重新计算当前的重力分量以便输出稳定的欧拉角
    float vx = 2.0f * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    float vy = 2.0f * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    float vz = 1.0f - 2.0f * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // 计算 Pitch: 使用 atan2 能跨越 90 度死区
    eulerAngle->pitch = -atan2f(vx, sqrtf(vy * vy + vz * vz)) * RtA;
    // 计算 Roll: 实现了与 Pitch 的解耦
    eulerAngle->roll  = atan2f(vy, vz) * RtA;
    // 偏航角 (Yaw): 保持原有的积分逻辑
    float yaw_rate = (float)gyroAccel->gyro.imu660ra_gyro_z * (4000.0f / 65536.0f);
    //死区判断
    // 原 0.35°/s 过大：陀螺仪温漂让 bias 偏到 +0.2°/s 时，
    // 死区会让正向噪声超阈被积分、负向被屏蔽，造成单边累积"假漂移"
    // 降到 0.05°/s：只压最底层传感器噪声，让对称噪声自然平均掉
    if(fabs(yaw_rate) > 0.05f) {
        eulerAngle->yaw += yaw_rate * dt;
    }

    // 10. 记录 Z 轴净加速度
    normAccz = gyroAccel->accel.imu660ra_acc_x * vx + 
               gyroAccel->accel.imu660ra_acc_y * vy + 
               gyroAccel->accel.imu660ra_acc_z * vz;
}

float Common_IMU_GetNormAccZ(void) { return normAccz; }