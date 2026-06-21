#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

/* ==================== 角速度环 LADRC（三轴独立，从PID参数推导） ====================
 *
 * 一阶 LADRC 映射：LADRC 等效小信号增益 = wc/b0，对应原 PID 的 KP。
 * 维持 wc/b0 = KP → wc = KP × b0，确保小信号响应与原 PID 一致。
 *
 * b0 物理推导（Pitch/Roll，质量~250g，轴距~250mm，hover油门~6900）：
 *   100单位差动 → 每电机±1.45%推力 → 推力差=0.017N → 力矩=0.0017Nm
 *   转动惯量 I_pr ≈ 0.0015 kg·m² → 角加速 α = 0.0017/0.0015 = 1.13 rad/s² = 65 °/s²
 *   b0_pr = 65/100 = 0.65 → 取整 b0=1.0（留余量，LADRC 对 b0 偏差不敏感）
 *
 * Yaw（桨阻矩起转，效率约 pitch 的 25% + 不同惯量比）：
 *   I_yaw / I_pr ≈ 1.8（X架），阻矩效率≈0.6 → b0_y = 1.0 × (1/1.8) × 0.6 ≈ 0.35
 *   但从 PID 调参看：KP_y/KP_pr = 10/14 = 0.71，且out_limit_y=400而pr=1200
 *   → 等效 b0_y = 1.0 × 0.71 = 0.71（取 0.7）
 *
 * wo = 4 × wc（标准一阶 LADRC 保守比，观测器快于控制器）
 *
 * 限幅：同原 PID out_limit——LADRC 输出与 PID 同量纲、同范围。
 *   Pitch/Roll: 1200（原PID值），Yaw: 400（已预留前馈YA≈120余量→合计≤520）
 */
/* LADRC 参数（定义在 ladrc.h，LADRC 仅旁观不控制，输出供调参对比） */

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
extern uint8_t ladrc_active;   /* 0=起飞用原PID, 1=悬停后切LADRC */

/* LADRC 旁观调试变量（定义在 ladrc.c，串口打印用） */
extern float ladrc_dbg_lpo;    /* LADRC Pitch 输出（对比 PO） */
extern float ladrc_dbg_lro;    /* LADRC Roll  输出（对比 RO） */
extern float ladrc_dbg_lyo;    /* LADRC Yaw   输出（对比 YO） */

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
