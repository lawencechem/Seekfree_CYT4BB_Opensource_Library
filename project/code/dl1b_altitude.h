#ifndef _DL1B_ALTITUDE_H_
#define _DL1B_ALTITUDE_H_

#include "zf_common_headfile.h"
#include "pid.h"

// ========================
// 自动飞行状态机枚举
// ========================
typedef enum {
    ALT_IDLE = 0,     // 怠速/锁定状态 (电机停转)
    ALT_TAKEOFF,      // 自主推油门起飞阶段
    ALT_HOLD,         // 自动定高悬停阶段
    ALT_LANDING       // 自主缓慢下降并检测触地
} Alt_State_e;

// ========================
// 定高模块对外接口与变量
// ========================
extern Alt_State_e flight_state;   // 当前飞行状态
extern PID_t pid_alt_pos;          // 高度位置环 (外环)
extern PID_t pid_alt_vel;          // 高度速度环 (内环)             

extern float target_height_cm;     // 目标悬停高度 (厘米)
extern float current_height_cm;    // 滤波后的真实高度 (厘米)
extern float current_speed_z;      // 融合计算出的 Z 轴速度 (cm/s，向上为正)
extern volatile float throttle_output;      // 最终输出给混控矩阵的总油门
extern float dbg_alt_err;          // 高度误差
extern float dbg_target_vel;       // 位置环输出目标速度
extern float dbg_vel_err;          // 速度误差
extern float dbg_vel_out;          // 速度环输出(未加悬停油门)
extern float dbg_thr_base;         // 油门基值（悬停或起飞基值）
extern float dbg_thr_alt;          // 高度速度环输出的油门修正量，不含倾角/电压补偿
extern float target_speed_z;       // 定高位置环输出的目标 Z 速度
extern float dbg_alt_pos_out;      // 高度位置环输出，等价于 target_speed_z
extern float dbg_alt_vel_out;      // 高度速度环输出，未加悬停油门
extern float alt_out;              // 同 dbg_thr_alt：正数加油门，负数减油门
extern float dbg_thr_precomp;      // 高度环修正后的油门，未做倾角/电压补偿
extern float dbg_thr_after_tilt;   // 倾角补偿后的油门，未做电压补偿
extern float dbg_thr_after_batt;   // 电压补偿后的油门，未限幅
extern float dbg_batt_delta;       // 电压补偿增加的油门量

// ========================
// 悬停油门与限幅宏定义 (根据你的 RS2205 + 3S 配置)
// ========================
#define THR_MOTOR_START    1500.0f  // 电机刚开始旋转的起步油门
// #define THR_HOVER_DEFAULT  6600.0f  // 旧值：对 0.5m 初测偏高，容易冲高
// #define THR_MAX_OUTPUT     9000.0f  // 旧值：上限偏大，初测阶段先收住
// #define THR_MIN_OUTPUT     5600.0f  // 旧值：最低油门偏高，掉高时不够柔和
#define THR_HOVER_DEFAULT  6900.0f  // 1m 定高测试：比固定油门略高，保留爬升余量  7050
#define THR_MAX_OUTPUT     8300.0f  // 公共油门上限：保护姿态纠偏余量，非电机上限
#define THR_MIN_OUTPUT     5200.0f  // 飞行中最低油门保守下探，避免高度环太硬

#define CLIMB_UP_MAX_SPEED   18.0f // 12→18：加快爬升
#define CLIMB_DOWN_MAX_SPEED -30.0f // 原来 -45，先降到 -30

// 自动任务目标：悬停高度与保持时间
#define ALT_HOLD_TARGET_CM   120.0f  // 定高/跟车目标：1.2m
#define ALT_HOLD_TIME_S      999.0f
#define ALT_HOLD_BAND_CM     5.0f    // 原来 8，先收紧一点

// 安全保护参数
#define ALT_TOF_LOST_FAILSAFE_S    0.5f   // TOF 丢失超过该时间触发保护
#define ALT_TAKEOFF_TOF_GRACE_S    0.8f   // 起飞阶段 TOF 丢失后的缓冲时间
#define ALT_LAND_DETECT_CM         18.0f  // 原来 15，稍微放宽落地检测
#define ALT_LAND_CONFIRM_S         0.2f   // 连续满足触地条件的确认时间

// 函数声明
void Altitude_System_Init(void);
void Altitude_Control_Task(float dt, uint8 tof_has_new);
float Altitude_Tilt_Compensate(float base_thr, float pitch_deg, float roll_deg);

#endif
