#ifndef MPU660RA_H_
#define MPU660RA_H_

#include "zf_common_headfile.h"

// 陀螺仪数据  16位ADC的值
typedef struct
{
    int16 imu660ra_gyro_x; // 往右飞为正   表示横滚角
    int16 imu660ra_gyro_y; // 向前飞转动为正 表示俯仰角
    int16 imu660ra_gyro_z; // 逆时针转动为正  表示偏航角
} Gyro_struct;

// 加速度计数据  16位ADC的值(原始数据)
typedef struct
{
    int16 imu660ra_acc_x; // 往前为正
    int16 imu660ra_acc_y; // 往左为正
    int16 imu660ra_acc_z; // 朝上的加速度为正
} Accel_struct;

typedef struct
{
    Gyro_struct gyro;
    Accel_struct accel;
} Gyro_Accel_Struct;

// 解算得到的欧拉角
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;

extern Accel_struct last_accel;
extern Gyro_struct last_gyro;
extern Gyro_Accel_Struct gyro_accel_data;
extern Euler_struct current_euler;
extern Gyro_Accel_Struct filtered_data;

// 陀螺仪/加速度计零偏（标定后保存，每次读 IMU 时减掉）
extern int32 acc_x_offset;
extern int32 acc_y_offset;
extern int32 acc_z_offset;
extern int32 gyro_x_offset;
extern int32 gyro_y_offset;
extern int32 gyro_z_offset;

void IMU660RA_Get_Acc(Accel_struct *acc);
void IMU660RA_Get_Gyro(Gyro_struct *gyro);
void Int_MPU6050_Get_Data(Gyro_Accel_Struct *data);
void Int_MPU6050_calculate_offset(void);


#endif