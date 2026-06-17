#include "mpu660ra.h"
#include "zf_common_headfile.h" // 包含所有库文件
#include "zf_device_imu660ra.h" // 确保包含这个，它是寄存器读写的源头

Gyro_struct  last_gyro  = {0};
Accel_struct last_accel = {0};

Euler_struct current_euler = {0}; // 存放解算后的欧拉角
Gyro_Accel_Struct filtered_data = {0}; // 存放滤波后的六轴数据

// 保存偏移量的值
int32 acc_x_offset = 0;
int32 acc_y_offset = 0;
int32 acc_z_offset = 0;

int32 gyro_x_offset = 0;
int32 gyro_y_offset = 0;
int32 gyro_z_offset = 0;

void IMU660RA_Get_Acc(Accel_struct *acc)
{
    uint8 dat[6];

    // 1. 调用逐飞库底层函数读取寄存器
    imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);

    // 2. 将合成后的原始数据直接存入指针指向的内存位置
    // 这样数据存放位置就完全取决于调用时传入的变量地址
    acc->imu660ra_acc_x = (int16)(((uint16)dat[1] << 8 | dat[0])) - acc_x_offset;
    acc->imu660ra_acc_y = (int16)(((uint16)dat[3] << 8 | dat[2])) - acc_y_offset;
    acc->imu660ra_acc_z = (int16)(((uint16)dat[5] << 8 | dat[4])) - acc_z_offset;
}

void IMU660RA_Get_Gyro(Gyro_struct *gyro)
{
    uint8 dat[6];

    // 1. 读取陀螺仪寄存器地址（IMU660RA_GYRO_ADDRESS）
    imu660ra_read_registers(IMU660RA_GYRO_ADDRESS, dat, 6);

    // 2. 对应关系与数据搬运
    // 同时也保留了你提供的逐飞库自带的硬补偿数值：+3, -1, -5
    gyro->imu660ra_gyro_x = (int16)(((uint16)dat[1] << 8 | dat[0])) - gyro_x_offset;  //+ 3
    gyro->imu660ra_gyro_y = (int16)(((uint16)dat[3] << 8 | dat[2])) - gyro_y_offset;  //- 1
    gyro->imu660ra_gyro_z = (int16)(((uint16)dat[5] << 8 | dat[4])) - gyro_z_offset;  //- 5
}

void Int_MPU6050_Get_Data(Gyro_Accel_Struct *data)
{
    IMU660RA_Get_Gyro(&data->gyro);
    IMU660RA_Get_Acc(&data->accel);
}

//void Int_MPU6050_Get_Data(Gyro_Accel_Struct *data)
//{
//    Gyro_struct  raw_g;
//    Accel_struct raw_a;
//    
//    IMU660RA_Get_Gyro(&raw_g);
//    IMU660RA_Get_Acc(&raw_a);
//
//    // 1. 陀螺仪方向取反：抬头变正，右滚变正
//    data->gyro.imu660ra_gyro_x = -raw_g.imu660ra_gyro_x; 
//    data->gyro.imu660ra_gyro_y = -raw_g.imu660ra_gyro_y;
//    data->gyro.imu660ra_gyro_z =  raw_g.imu660ra_gyro_z;
//
//    // 2. 加速度计方向也必须同步取反（非常重要，否则 Kp 补偿会反向拽）
//    data->accel.imu660ra_acc_x = -raw_a.imu660ra_acc_x;
//    data->accel.imu660ra_acc_y = -raw_a.imu660ra_acc_y;
//    data->accel.imu660ra_acc_z =  raw_a.imu660ra_acc_z;
//}


//在初始化MPU6050完成之后 对MPU6050进行零偏校准
void Int_MPU6050_calculate_offset(void)
{
    // 1. 等待飞机停放平稳
    system_delay_ms(2000);
  // 判断飞机是否停放平稳的标准: 前后两次加速度的值差值小于200 连续100次
    Accel_struct current_accel = {0};
    Accel_struct last_accel = {0};
    uint8_t count = 0;
    
    // 初始化 offset 为 0，确保校准时拿到的是真实的原始偏差
    acc_x_offset = acc_y_offset = acc_z_offset = 0;
    gyro_x_offset = gyro_y_offset = gyro_z_offset = 0;
    
    IMU660RA_Get_Acc(&last_accel);
    
    while (count < 100)
    {
        IMU660RA_Get_Acc(&current_accel);
        // 判断飞机是否平稳 选用的参数过小 会造成一直无法判断为平稳
        if (abs(current_accel.imu660ra_acc_x - last_accel.imu660ra_acc_x) < 200 && abs(current_accel.imu660ra_acc_y - last_accel.imu660ra_acc_y) < 200 && abs(current_accel.imu660ra_acc_z - last_accel.imu660ra_acc_z) < 200)
        {
            count++;
        }
        else
        {
            count = 0;
        }
        last_accel = current_accel;
        system_delay_ms(5);
    }
    // 2. 飞机已经平稳 开始进行零偏校准
    Gyro_Accel_Struct gyro_accel_data = {0};
    int32 acc_x_sum = 0;
    int32 acc_y_sum = 0;
    int32 acc_z_sum = 0;

    int32 gyro_x_sum = 0;
    int32 gyro_y_sum = 0;
    int32 gyro_z_sum = 0;
    
    // 采样次数从 100 (600ms) 提到 500 (3s)：让噪声平均更充分，
    // 单次标定残留 bias 从 ±0.1°/s 量级降到 ±0.04°/s 量级。
    // 这是治 yaw 慢漂的关键之一，配合起飞前重新标定一起用。
    // 注意：循环变量必须用 uint16，uint8 装不下 500。
    #define IMU_BIAS_CAL_SAMPLES   500
    for (uint16_t i = 0; i < IMU_BIAS_CAL_SAMPLES; i++)
    {
        // 重新读取加速度和角速度
        Int_MPU6050_Get_Data(&gyro_accel_data);
        acc_x_sum += (gyro_accel_data.accel.imu660ra_acc_x - 0);
        acc_y_sum += (gyro_accel_data.accel.imu660ra_acc_y - 0);
        // Z轴加速度的初始化应该就是1g  => 量程是8g => 4096
        acc_z_sum += (gyro_accel_data.accel.imu660ra_acc_z - 4096);

        gyro_x_sum += (gyro_accel_data.gyro.imu660ra_gyro_x - 0);
        gyro_y_sum += (gyro_accel_data.gyro.imu660ra_gyro_y - 0);
        gyro_z_sum += (gyro_accel_data.gyro.imu660ra_gyro_z - 0);

        // 每次测量数据需要添加延迟  多次测量取平均值才有意义
        system_delay_ms(6);
    }

    acc_x_offset = acc_x_sum / IMU_BIAS_CAL_SAMPLES;
    acc_y_offset = acc_y_sum / IMU_BIAS_CAL_SAMPLES;
    acc_z_offset = acc_z_sum / IMU_BIAS_CAL_SAMPLES;

    gyro_x_offset = gyro_x_sum / IMU_BIAS_CAL_SAMPLES;
    gyro_y_offset = gyro_y_sum / IMU_BIAS_CAL_SAMPLES;
    gyro_z_offset = gyro_z_sum / IMU_BIAS_CAL_SAMPLES;
}