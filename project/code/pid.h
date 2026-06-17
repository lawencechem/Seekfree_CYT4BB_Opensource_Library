#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

// PID 参数结构体
typedef struct 
{
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float i_limit;    // 积分限幅
    float out_limit;  // 输出限幅
    float output;
    float last_current;      // 上次实际测量值
    float derivative;        // 滤波后的 D 项当前值
    float last_derivative;   // 滤波后的 D 项上次值
    float d_lpf_alpha;       // 低通滤波系数 (0.0 ~ 1.0)，越小滤波越强，延迟越大
    
} PID_t;

// 外部声明 PID 实例
extern PID_t pid_pitch_angle, pid_pitch_rate;
extern PID_t pid_roll_angle, pid_roll_rate;
extern PID_t pid_yaw_angle, pid_yaw_rate;
extern float p_out, r_out, y_out; // 增加这两行声明
extern float p_target_rate;
extern float y_limited;

extern float r_target_rate_dbg;
extern float y_target_rate_dbg;
extern float pitch_target;
extern float roll_target;
extern float target_yaw;


extern float dbg_pitch_trim;
extern float dbg_roll_trim;
extern float dbg_pitch_rate_fb;
extern float dbg_roll_rate_fb;
extern float dbg_yaw_rate_fb;

extern float pitch_zero_offset;
extern float roll_zero_offset;
extern float pitch_flight_trim;
extern float roll_flight_trim;
extern float yaw_bias_base;     // Yaw 前馈表的初始基准值，调表时作为参考，运行期不参与计算
extern float yaw_bias_adapt;    // 实际生效的 Yaw 反扭矩前馈：按当前油门在 yaw_bias_bp[] 表插值 + 慢速低通后的结果
extern uint8_t attitude_zero_ready;
extern uint8_t yaw_i_enable;

extern int16 dbg_m1_target;
extern int16 dbg_m2_target;
extern int16 dbg_m3_target;
extern int16 dbg_m4_target;
extern uint8_t dbg_takeoff_state;        // 起飞状态机：0=IDLE 1=PRESPIN 2=RAMP_LIMIT 3=FULL
extern float   dbg_trim_blend;           // 起飞阶段 trim 渐入因子：IDLE/PRESPIN=0，RAMP_LIMIT 内 0→1，FULL=1
extern uint8_t dbg_takeoff_abort;        // 起飞翻倒锁存：1=姿态角超过 TAKEOFF_ABORT_ANGLE 已锁电机，须回 ALT_IDLE 或油门<400 才解除
extern float   dbg_takeoff_trim_weight;  // 高度相关 trim 渐入权重（0~1），由当前高度对 TRIM_START/FULL_HEIGHT_CM 线性映射
extern float   dbg_yaw_bias_for_mix;     // 真正写入电机混控的 Yaw 前馈 = yaw_bias_adapt × 电池电压补偿系数
extern float   dbg_yaw_takeoff_weight;   // Yaw 前馈起飞渐入权重，目前固定 1.0（实测渐入反而不稳，保留全量前馈）


// 函数声明
float f_limit(float val, float min, float max);
float Yaw_Bias_By_Throttle(float comp_throttle);
void PID_Params_Init(void);
void Attitude_Zero_Calibrate(uint16_t sample_ms);
float PID_Compute(PID_t *pid, float target, float current, float dt);
void Motor_Control_Mixing(float throttle);

#endif
