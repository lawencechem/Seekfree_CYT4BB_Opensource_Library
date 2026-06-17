#include "filter.h"

// 新值的系数  值越小  低通滤波的效果越强
#define ALPHA 0.15       //一阶低通滤波 指数加权系数 

Gyro_struct filtered_gyro = {0};

//@description: 一阶低通滤波
//@param {int16_t} newValue 需要滤波的值 
//@param {int16_t} preFilteredValue 上一次滤波过的值
//@return {*}

int16 Common_Filter_LowPass(int16 newValue, int16 preFilteredValue)
{
  float result = (float)newValue * ALPHA + (float)preFilteredValue * (1.0f - ALPHA);  
  return (int16)(result + 0.5f);
}

//读取原始数据 -> 滤波 -> 更新到全局变量
void Update_Gyro_Filter(Gyro_struct *raw_gyro)
{
    filtered_gyro.imu660ra_gyro_x = Common_Filter_LowPass(raw_gyro->imu660ra_gyro_x, filtered_gyro.imu660ra_gyro_x);
    filtered_gyro.imu660ra_gyro_y = Common_Filter_LowPass(raw_gyro->imu660ra_gyro_y, filtered_gyro.imu660ra_gyro_y);
    filtered_gyro.imu660ra_gyro_z = Common_Filter_LowPass(raw_gyro->imu660ra_gyro_z, filtered_gyro.imu660ra_gyro_z);
}



//卡尔曼滤波参数 
KalmanFilter_Struct kfs[3] = 
{
    {0.02, 0, 0, 0, 0.001, 2},   // X轴   0.543
    {0.02, 0, 0, 0, 0.001, 2},   // Y轴   0.543
    {0.02, 0, 0, 0, 0.001, 2}    // Z轴   0.543
};

Accel_struct filtered_accel = {0};   //存储滤波后的结果

//卡尔曼滤波核心算法
double Common_Filter_KalmanFilter(KalmanFilter_Struct *kf, double input)
{
    // 预测步骤
    kf->Now_P = kf->LastP + kf->Q;
    
    // 更新步骤
    kf->Kg = kf->Now_P / (kf->Now_P + kf->R);
    kf->out = kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1 - kf->Kg) * kf->Now_P;
    
    return kf->out;
}

void Update_Accel_KalmanFilter(Accel_struct *raw_accel)
{
    // 依次对 X, Y, Z 轴进行卡尔曼滤波，并强制转换为 int16 存入结果变量
    filtered_accel.imu660ra_acc_x = (int16)Common_Filter_KalmanFilter(&kfs[0], (double)raw_accel->imu660ra_acc_x);
    filtered_accel.imu660ra_acc_y = (int16)Common_Filter_KalmanFilter(&kfs[1], (double)raw_accel->imu660ra_acc_y);
    filtered_accel.imu660ra_acc_z = (int16)Common_Filter_KalmanFilter(&kfs[2], (double)raw_accel->imu660ra_acc_z);
}