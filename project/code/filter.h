#ifndef FILTER_H_
#define FILTER_H_

#include "zf_common_headfile.h"

//低通滤波
int16 Common_Filter_LowPass(int16 newValue, int16 preFilteredValue);
void Update_Gyro_Filter(Gyro_struct *raw_gyro);

extern Gyro_struct filtered_gyro;        // 声明滤波后的全局变量，供 main.c 打印使用

//卡尔曼滤波 (Q为过程噪声，R为测量噪声。如果觉得波形毛刺多，可以尝试增大 R)
typedef struct
{
    float LastP; // 上一时刻的状态方差（或协方差）
    float Now_P; // 当前时刻的状态方差（或协方差）
    float out;   // 滤波器的输出值，即估计的状态
    float Kg;    // 卡尔曼增益，用于调节预测值和测量值之间的权重
    float Q;     // 过程噪声的方差，反映系统模型的不确定性
    float R;     // 测量噪声的方差，反映测量过程的不确定性
} KalmanFilter_Struct;

extern KalmanFilter_Struct kfs[3];   // 声明卡尔曼滤波器组（对应 X, Y, Z 三轴）
extern Accel_struct filtered_accel;  // 声明滤波后的加速度变量，供 main 打印或姿态解算使用

double Common_Filter_KalmanFilter(KalmanFilter_Struct *kf, double input);
void Update_Accel_KalmanFilter(Accel_struct *raw_accel);

#endif