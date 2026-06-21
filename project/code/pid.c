#include "pid.h"
#include "mpu660ra.h"      // 引用你的 current_euler 等变量
#include "small_driver_uart_control.h"
#include "dl1b_altitude.h"
#include "battery_comp.h"
#include "UP_FLOW_302.h"
#include "ladrc.h"
PID_t pid_pitch_angle, pid_pitch_rate;
PID_t pid_roll_angle, pid_roll_rate;
PID_t pid_yaw_angle, pid_yaw_rate;

float target_yaw = 0.0f;

// float pitch_trim = -1.0f;   // 旧版：固定姿态 trim
// float roll_trim  = 1.2f;    // 旧版：固定姿态 trim
float pitch_zero_offset = 0.0f;
float roll_zero_offset  = 0.0f;

// 飞行微调量：位置环介入后不需要了，设0交给位置环自动补偿
float pitch_flight_trim = 0.0f;
float roll_flight_trim  = 0.0f;

uint8_t attitude_zero_ready = 0;
uint8_t yaw_i_enable = 1;   // 起飞低高度/大爬升速度时关闭 Yaw 积分，避免扰动被过早积进去
float yaw_bias_base = 110.0f;  // 表格初始基准值，后续手动调表时作为参考。133.5   112
float yaw_bias_adapt = 110.0f; // 根据当前混控油门插值得到的 Yaw 前馈补偿。133.5

// ==================== Yaw 油门前馈表 ====================
// 思路：
// 1) Yaw PID 负责动态纠偏；
// 2) yaw_bias_adapt 负责不同油门点下的静态反扭矩补偿；
// 3) 先关闭自学习，只根据日志里的 YI 手动改 yaw_bias_bp[]。
//#define YAW_FF_N 5
//
//static const float yaw_thr_bp[YAW_FF_N] = {
//    6200.0f, 6400.0f, 6600.0f, 6800.0f, 7000.0f
//};

// static float yaw_bias_bp[YAW_FF_N] = {
//     133.0f, 134.0f, 134.3f, 134.5f, 134.6f
// };  
//static float yaw_bias_bp[YAW_FF_N] = {
//    148.0f, 150.0f, 151.0f, 152.0f, 153.0f
//};
 

//static float yaw_bias_bp[YAW_FF_N] = {
//    124.0f, 125.0f, 125.5f, 126.0f, 126.5f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    120.0f, 121.0f, 121.5f, 122.0f, 122.5f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//            129.0f, 130.0f, 130.5f, 131.0f, 131.5f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//            134.0f, 135.0f, 136.0f, 137.0f, 138.0f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    133.0f, 134.5f, 136.0f, 137.5f, 140.0f
//};

#define YAW_FF_N 7

//static const float yaw_thr_bp[YAW_FF_N] = {
//    6200.0f, 6400.0f, 6600.0f, 6800.0f, 7000.0f, 7200.0f, 7400.0f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    133.0f, 134.5f, 136.0f, 137.8f, 140.8f, 143.0f, 145.0f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    128.0f, 129.5f, 131.0f, 132.5f, 134.0f, 136.0f, 138.0f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    120.0f, 122.0f, 124.0f, 126.0f, 127.0f, 128.0f, 130.0f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    112.0f, 114.0f, 116.0f, 118.0f, 119.0f, 120.0f, 122.0f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    116.0f, 118.0f, 120.0f, 122.0f, 123.0f, 124.0f, 126.0f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    114.0f, 116.0f, 118.0f, 120.0f, 121.0f, 122.0f, 124.0f
//};

//static float yaw_bias_bp[YAW_FF_N] = {
//    112.0f, 114.0f, 116.0f, 118.0f, 119.0f, 120.0f, 122.0f
//};
//static float yaw_bias_bp[YAW_FF_N] = {
//    138.0f, 142.0f, 146.0f, 150.0f, 154.0f, 158.0f, 162.0f

static const float yaw_thr_bp[YAW_FF_N] = {
    6000.0f, 6200.0f, 6400.0f, 6600.0f, 6800.0f, 7000.0f, 7200.0f
};

static float yaw_bias_bp[YAW_FF_N] = {
    85.0f, 86.0f, 87.0f, 88.0f, 89.0f, 90.0f, 91.0f
};   /* 原110~120偏大→YI=-89，降~22%让YI≈0 */
//};

static float yaw_bias_filt = 85.0f;

#define MOTOR_MIN  3200   // 确保电机始终正转的最小油门
#define MOTOR_MAX  9500  // 预留一点上限，防止电调饱和失步
// YAW 混控方向开关：+1 保持当前方向，-1 反向（用于快速校正旋转方向）
#define YAW_MIX_SIGN (1.0f)
// Yaw 反扭矩前馈开关：1=用 yaw_bias 油门前馈表(老逻辑)  0=关掉前馈，纯靠PID(诊断/重构用)
// 实验目的：关掉前馈后看 YAW 是"停在一个固定偏角"还是"继续慢慢转"——
//   停住 → 前馈(随油门变)就是慢转元凶，下一步把航向交给积分自整定；
//   继续转 → 存在真实大力矩，需要给积分足够授权去吃掉它。
#define YAW_FF_ENABLE (1)
// 偏航航向锁定开关：1=正常锁航向(外环+内环)  0=路A诊断(关外环，只做角速度阻尼)
#define YAW_HEADING_HOLD   (1)
// ==================== 起飞保护流程参数 ====================
// 这部分专门处理“贴地起转 -> 离地初期 -> 完整姿态控制”的过渡，不和常规 PID 参数混在一起。
// 目的：
// 1) 先让四个电机用完全相同的 duty 同步预转，避免某个电机晚起转；
// 2) 起飞初期慢慢加油，避免基础油门突然跨过起飞点；
// 3) 起飞初期让 trim 从 0 慢慢混入，避免 roll_flight_trim 在贴地阶段直接撬机体；
// 4) 起飞初期限制 P/R/Y 修正权重，防止还没离地时 PID 把某个电机拉得过高或过低。
#define TAKEOFF_DT_MS              2U
#define TAKEOFF_PRESPIN_MS         600U
#define TAKEOFF_RAMP_MS            1200U
#define MOTOR_IDLE_DUTY            3500.0f
#define TAKEOFF_P_OUT_LIMIT        250.0f
#define TAKEOFF_R_OUT_LIMIT        250.0f
#define TAKEOFF_Y_OUT_LIMIT        120.0f
#define TAKEOFF_ABORT_ANGLE        30.0f

#define TRIM_ENABLE_HEIGHT_CM      8.0f    //25  15
#define TRIM_FULL_HEIGHT_CM        30.0f    //70  45

#define TAKEOFF_STATE_IDLE         0U
#define TAKEOFF_STATE_PRESPIN      1U
#define TAKEOFF_STATE_RAMP_LIMIT   2U
#define TAKEOFF_STATE_FULL         3U

/**
 * @brief 辅助限幅函数
 * @param val 输入值, min 下限, max 上限
 * @return 限制后的输出值
 */
float f_limit(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// ============ 动力余量保护 ============
// 按当前公共油门的剩余余量，动态缩放姿态输出，防止某个电机冲到 MOTOR_MAX 或掉到 MOTOR_MIN。
// 优先级：先保 pitch/roll(防翻)，余量不够时才砍偏航(宁可转一点，也不可翻)。
#define ATT_HEADROOM_RESERVE   250.0f   // 给硬限幅留的缓冲

static void Attitude_Mix_Headroom_Limit(float base_duty,
                                        float *p_out, float *r_out, float *y_mix)
{
    float up_headroom   = (float)MOTOR_MAX - base_duty - ATT_HEADROOM_RESERVE;
    float down_headroom = base_duty - (float)MOTOR_MIN - ATT_HEADROOM_RESERVE;
    float usable = (up_headroom < down_headroom) ? up_headroom : down_headroom;
    if (usable < 0.0f) usable = 0.0f;

    // 先把余量分给 pitch/roll
    float pr_demand = fabsf(*p_out) + fabsf(*r_out);
    if (pr_demand > usable && pr_demand > 1.0f)
    {
        // 连 pitch/roll 都放不下：等比缩 p/r，偏航砍到 0（保命优先）
        float prs = usable / pr_demand;
        *p_out *= prs;
        *r_out *= prs;
        *y_mix  = 0.0f;
        return;
    }

    // pitch/roll 放得下，剩余的留给偏航
    float y_usable = usable - pr_demand;
    float y_demand = fabsf(*y_mix);
    if (y_demand > y_usable && y_demand > 1.0f)
    {
        *y_mix *= (y_usable / y_demand);
    }
}

static float interp1_float(const float *x, const float *y, int n, float xi)
{
    if (xi <= x[0]) {
        return y[0];
    }
    if (xi >= x[n - 1]) {
        return y[n - 1];
    }

    for (int i = 0; i < n - 1; i++) {
        if (xi >= x[i] && xi <= x[i + 1]) {
            float t = (xi - x[i]) / (x[i + 1] - x[i]);
            return y[i] + t * (y[i + 1] - y[i]);
        }
    }

    return y[n - 1];
}

/**
 * @brief 根据当前进入混控的油门获取 Yaw 前馈补偿
 * @note 自学习暂时关闭。后续根据日志中不同油门段的 YI 平均值，手动微调 yaw_bias_bp[]。
 */
float Yaw_Bias_By_Throttle(float comp_throttle)
{
    float target_bias;

    comp_throttle = f_limit(comp_throttle, yaw_thr_bp[0], yaw_thr_bp[YAW_FF_N - 1]);
    target_bias = interp1_float(yaw_thr_bp, yaw_bias_bp, YAW_FF_N, comp_throttle);

    // 防止表值误填过大导致偏航突然抢权。
    target_bias = f_limit(target_bias, 80.0f, 200.0f);

    // 慢速低通，避免高度环油门变化时 yaw 前馈跟着突变。
    yaw_bias_filt += 0.03f * (target_bias - yaw_bias_filt);
    yaw_bias_adapt = yaw_bias_filt;

    return yaw_bias_adapt;
}

/* ==================== 角速度环 LADRC（定义在Attitude_PID_History_Reset之前） ====================
 * 一阶 LADRC 替代 PID_Compute_Rate。
 * ESO 在线估计角加速度扰动并主动抵消。
 * 三轴参数独立（Pitch/Roll/Yaw），来自各轴 PID 反映的物理差异。
 */
/* LADRC 旁观标志（在 dl1b_altitude.c HOLD 切入时置 1，仅记录状态） */
uint8_t ladrc_active = 0;

/**
 * @brief 清空姿态 PID 历史项
 * @note 起飞预转/落地/保护时调用，避免飞机还被地面支撑时 PID 积分”憋劲”。
 */
static void Attitude_PID_History_Reset(void)
{
    pid_pitch_rate.integral = 0.0f; pid_pitch_angle.integral = 0.0f;
    pid_roll_rate.integral  = 0.0f; pid_roll_angle.integral  = 0.0f;
    pid_yaw_rate.integral   = 0.0f; pid_yaw_angle.integral   = 0.0f;

    pid_pitch_rate.last_current = 0.0f;
    pid_roll_rate.last_current  = 0.0f;
    pid_yaw_rate.last_current   = 0.0f;

    pid_pitch_rate.last_derivative = 0.0f;
    pid_roll_rate.last_derivative  = 0.0f;
    pid_yaw_rate.last_derivative   = 0.0f;

    pid_pitch_rate.derivative = 0.0f;
    pid_roll_rate.derivative  = 0.0f;
    pid_yaw_rate.derivative   = 0.0f;

    ladrc_reset();
}

/**
 * @brief 上电静态姿态零点校准（机体放平且保持静止）
 * @param sample_ms 采样时长（毫秒），建议 1000ms
 */
void Attitude_Zero_Calibrate(uint16_t sample_ms)
{
    float sum_pitch = 0.0f;
    float sum_roll  = 0.0f;
    uint16_t cnt = 0;

    // 校准期间确保电机关闭
    small_driver_set_duty(0, 0, 0, 0);

    // 打印 boot 阶段标定的零偏（imu660ra_init 内部 Int_MPU6050_calculate_offset 标的）
    // 用户可以判断 bias 量级是否合理：
    //   gz 在 ±30 LSB（≈ ±0.03 °/s）内 → 优秀
    //   gz 在 ±100 LSB 内 → 可接受
    //   gz > ±200 → 标定时不稳，建议复位重来
    printf("\r\n==== Boot bias (after imu660ra_init): "
           "gx=%ld gy=%ld gz=%ld | ax=%ld ay=%ld az=%ld ====\r\n",
           (long)gyro_x_offset, (long)gyro_y_offset, (long)gyro_z_offset,
           (long)acc_x_offset,  (long)acc_y_offset,  (long)acc_z_offset);

    // 注：原本想在这里再调一次 Int_MPU6050_calculate_offset() 来捕获"温热状态"
    //     的 bias，但该函数会和 PIT_CH0(2ms) 里的 IMU 读取在 I²C 总线上互相干扰，
    //     导致静止检查死循环。暂保留 boot 标定结果，温漂残留靠后续策略治理。

    // 先等姿态解算稳定
    system_delay_ms(300);

    for (uint16_t t = 0; t < sample_ms; t += 10)
    {
        sum_pitch += current_euler.pitch;
        sum_roll  += current_euler.roll;
        cnt++;
        system_delay_ms(10);
    }

    if (cnt > 0)
    {
        pitch_zero_offset = sum_pitch / cnt;
        roll_zero_offset  = sum_roll  / cnt;
    }

    attitude_zero_ready = 1;

    // 清 PID 历史，避免校准前状态带入起飞
    Attitude_PID_History_Reset();

    target_yaw = current_euler.yaw;
}

/**
 * @brief PID 参数初始化
 * @note 建议调试顺序：先调内环(rate) P，再调外环(angle) P，最后微调 D
 */
void PID_Params_Init(void) 
{
    
    // --- 俯仰轴 (Pitch) ---
    // 内环：负责对抗震动和快速纠偏
    pid_pitch_rate.kp = 14.0f; //15以下  12  8（偏软）14（还行，但有点硬） 11  8.6
    pid_pitch_rate.ki = 0.04f; //0.2
    pid_pitch_rate.kd = 0.022f; //0.05  0.03（软）0.008(会侧翻) 0.035 0.022
    pid_pitch_rate.i_limit = 350.0f;       // 积分限幅，防止起飞前“憋气”过猛
    pid_pitch_rate.out_limit = 1200.0f;   // 原 1500
    pid_pitch_rate.d_lpf_alpha = 0.15f;    // 【新增】D项低通滤波系数 (0.2~0.5比较合适)0.3
    
    // 外环：负责把飞机拉回水平
    pid_pitch_angle.kp = 4.2f; //4.5，4（软）7  6
    pid_pitch_angle.ki = 0.00f; //0.05 
    pid_pitch_angle.kd = 0.0f;
    pid_pitch_angle.i_limit = 0.0f;      // 外环积分限幅 250  500
    pid_pitch_angle.out_limit = 60.0f;  // 90→60：回退(手拿数据无效)

    // --- 横滚轴 (Roll) ---
    // 内环：负责对抗震动和快速纠偏
    pid_roll_rate.kp = 13.5f;  //13.0  10.5  9.4
    pid_roll_rate.ki = 0.05f; //1.5  0.2
    pid_roll_rate.kd = 0.025f;//0.008 0.035  0.04
    pid_roll_rate.i_limit = 300.0f;       // 积分限幅，防止起飞前“憋气”过猛
    pid_roll_rate.out_limit = 1200.0f;    // 原 1600
    pid_roll_rate.d_lpf_alpha = 0.15f;
    
    // 外环：负责把飞机拉回水平
    pid_roll_angle.kp = 4.2f; //6  5  5.5
    pid_roll_angle.ki = 0.00f;  //0.05
    pid_roll_angle.kd = 0.0f;
    pid_roll_angle.i_limit = 0.0f;//250  500
    pid_roll_angle.out_limit = 60.0f; // 90→60：回退

    // --- 偏航轴 (Yaw) ---
    // 外环：航向角锁定 (Heading Hold)
    pid_yaw_angle.kp = 1.8f;        // 1.12→1.8，加强YAW锁定
    pid_yaw_angle.ki = 0.0f;       // 外环不需要 I
    pid_yaw_angle.kd = 0.0f;
    pid_yaw_angle.i_limit = 0.0f;
    pid_yaw_angle.out_limit = 55.0f; // 80→55，减少外环过冲防振荡

    // 内环：角速度控制
    pid_yaw_rate.kp = 10.0f; // 7.0→10.0，加强YAW角速度响应，锁死航向
    pid_yaw_rate.ki = 0.10f; // 0.20→0.10，减半积分防windup振荡
    pid_yaw_rate.kd = 0.0f;
    pid_yaw_rate.i_limit = 100.0f; // 200→100，YI限幅收紧，防持续积累
    pid_yaw_rate.out_limit = 400.0f;  /* 520→400：给前馈留120余量。PID+前馈不超过520，anti-windup正确生效 */
    pid_yaw_rate.d_lpf_alpha = 1.0f;
}

/**
 * @brief 外环角度 PID (带最短路径和积分分离)
 */
float PID_Compute_Angle(PID_t *pid, float target, float current, float dt) 
{
    pid->error = target - current;

    // 偏航角越界处理 (过零点最短路径)
    if (pid->error < -180.0f) pid->error += 360.0f;
    if (pid->error >  180.0f) pid->error -= 360.0f;

    // 积分分离 (防饱和)
    if (fabs(pid->error) < 20.0f) {
        pid->integral += pid->error * dt;
        pid->integral = f_limit(pid->integral, -pid->i_limit, pid->i_limit);
    }

    // 外环通常不需要 D 项，直接算 PI
    float output = (pid->kp * pid->error) + (pid->ki * pid->integral);
    return f_limit(output, -pid->out_limit, pid->out_limit);
}

/**
 * @brief 内环角速度 PID 【融合开源代码精髓：微分先行 + 一阶低通滤波】
 */
float PID_Compute_Rate(PID_t *pid, float target, float current, float dt) 
{
    pid->error = target - current;
    
    // 积分分离：偏差太大时说明飞机正在剧烈翻滚，此时暂停积分，防爆震
    if (fabs(pid->error) < 60.0f) {
        pid->integral += pid->error * dt;
        pid->integral = f_limit(pid->integral, -pid->i_limit, pid->i_limit);
    }

    // 1. 微分先行：只对实际反馈求导，忽略 target 变化带来的冲击
    float raw_derivative = -(current - pid->last_current) / dt;
    pid->last_current = current; 

    // 2. 一阶低通滤波 (吸收高频噪声，保护电机)
    // 公式：本次滤波结果 = 上次结果 + alpha * (本次原始值 - 上次结果)
    pid->derivative = pid->last_derivative + pid->d_lpf_alpha * (raw_derivative - pid->last_derivative);
    pid->last_derivative = pid->derivative;

    // 3. 输出计算
    float output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * pid->derivative);
    return f_limit(output, -pid->out_limit, pid->out_limit);
}

/**
 * @brief 专为 YAW 轴内环定制的 PID (移除了积分分离障碍)
 */
float PID_Compute_Yaw_Rate(PID_t *pid, float target, float current, float dt)
{
    pid->error = target - current;

    // ★ 积分泄漏(×0.9997@500Hz→τ≈7s)：防止YI缓慢累积到饱和，
    //   又保留对抗持续偏置的能力（FF扛主力119，I只作微调）。
    pid->integral *= 0.9997f;

    // Yaw anti-windup：输出饱和且误差同向时停止加深积分
    float i_candidate = pid->integral;
    if (yaw_i_enable && fabs(pid->error) < 180.0f) {
        i_candidate += pid->error * dt;
        i_candidate = f_limit(i_candidate, -pid->i_limit, pid->i_limit);
    }

    float unsat_out = (pid->kp * pid->error) + (pid->ki * i_candidate);
    float sat_out = f_limit(unsat_out, -pid->out_limit, pid->out_limit);

    if ((fabs(unsat_out - sat_out) < 1e-3f) || (pid->error * unsat_out < 0.0f)) {
        pid->integral = i_candidate;
    }

    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral);
    return f_limit(pid->output, -pid->out_limit, pid->out_limit);
}



/**
 * @brief 电机混控逻辑 (控制核心)
 * @param throttle 基础油门 
 */

float p_out = 0.0f;
float r_out = 0.0f;
float y_out = 0.0f;
float pitch_target = 0.0f;
float roll_target = 0.0f;
float p_target_rate = 0.0f;
float r_target_rate_dbg = 0.0f;
float y_target_rate_dbg = 0.0f;
float y_limited = 0.0f;

float dbg_pitch_trim = 0.0f;
float dbg_roll_trim  = 0.0f;
float dbg_pitch_rate_fb = 0.0f;
float dbg_roll_rate_fb  = 0.0f;
float dbg_yaw_rate_fb   = 0.0f;

int16 dbg_m1_target = 0;
int16 dbg_m2_target = 0;
int16 dbg_m3_target = 0;
int16 dbg_m4_target = 0;
uint8_t dbg_takeoff_state = TAKEOFF_STATE_IDLE;
float dbg_trim_blend = 0.0f;
uint8_t dbg_takeoff_abort = 0;
float dbg_takeoff_trim_weight = 0.0f;
float dbg_yaw_bias_for_mix = 0.0f;
float dbg_yaw_takeoff_weight = 1.0f;

// 温漂陀螺零偏重校准（定义在文件末尾，前向声明供 Motor_Control_Mixing 调用）
void Gyro_Bias_Recalibrate_2ms(int16 gx, int16 gy, int16 gz, float throttle);

void Motor_Control_Mixing(float throttle)
{
    static uint16_t boot_timer = 0;
    static uint16_t takeoff_elapsed_ms = 0;
    static uint8_t takeoff_state = TAKEOFF_STATE_IDLE;
    static uint8_t takeoff_abort_latched = 0;
    float takeoff_trim_weight = 0.0f;
    float final_trim_blend = 0.0f;
    float yaw_takeoff_weight = 1.0f;
    float yaw_bias_for_mix = 0.0f;
    
    // 上电缓启动保护
    if (boot_timer < 250) { 
        boot_timer++;
        p_out = 0; r_out = 0; y_out = 0;
        y_limited = 0;
        yaw_i_enable = 0;
        dbg_takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_trim_blend = 0.0f;
        dbg_takeoff_abort = 0;
        dbg_m1_target = 0; dbg_m2_target = 0; dbg_m3_target = 0; dbg_m4_target = 0;
        small_driver_set_duty(0, 0, 0, 0); 
        return; 
    }
    
    float base_duty = f_limit(throttle, 500.0f, 9000.0f);
    float p_rate_fb = -(float)filtered_data.gyro.imu660ra_gyro_y * 0.061f;
    float r_rate_fb = -(float)filtered_data.gyro.imu660ra_gyro_x * 0.061f;
    float y_rate_fb =  (float)filtered_data.gyro.imu660ra_gyro_z * 0.061f;

    // 温漂陀螺零偏后台重校准（每次ISR调用，仅在静止+低油门时更新）
    Gyro_Bias_Recalibrate_2ms(last_gyro.imu660ra_gyro_x,
                              last_gyro.imu660ra_gyro_y,
                              last_gyro.imu660ra_gyro_z,
                              throttle);

    // 起飞流程入口安全保险：
    // 起飞阶段不是等飞机翻到很大角度才保护，而是刚出现明显翻机趋势就中止起飞。
    // TAKEOFF_ABORT_ANGLE 建议先用 30 度；如果地面姿态很不平，先机械放平，不要放宽保护角。
    if (takeoff_state != TAKEOFF_STATE_FULL &&
        (fabs(current_euler.pitch) > TAKEOFF_ABORT_ANGLE ||
         fabs(current_euler.roll)  > TAKEOFF_ABORT_ANGLE))
    {
        // 起飞阶段一旦出现明显翻机趋势，锁存中止状态。
        // 固定油门模式下不能让它扶正后自动重新起飞，必须先回到 ALT_IDLE 或油门低于 400 才允许重启。
        takeoff_abort_latched = 1;
        dbg_takeoff_abort = 1;
        takeoff_elapsed_ms = 0;
        takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_trim_blend = 0.0f;

        p_out = 0.0f; r_out = 0.0f; y_out = 0.0f; y_limited = 0.0f;
        yaw_i_enable = 0;
        dbg_m1_target = 0; dbg_m2_target = 0; dbg_m3_target = 0; dbg_m4_target = 0;
        Attitude_PID_History_Reset();
        target_yaw = current_euler.yaw;
        small_driver_set_duty(0, 0, 0, 0);
        return;
    }

    // ==================== 起飞保护流程：独立于常规 PID 参数 ====================
    // 如果油门关闭或飞行状态回到 IDLE，整个起飞流程复位，下次再从“同步预转”重新开始。
    if (flight_state == ALT_IDLE || throttle < 400.0f)
    {
        takeoff_abort_latched = 0;
        dbg_takeoff_abort = 0;
        takeoff_elapsed_ms = 0;
        takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_trim_blend = 0.0f;

        p_out = 0.0f; r_out = 0.0f; y_out = 0.0f; y_limited = 0.0f;
        yaw_i_enable = 0;
        dbg_m1_target = 0; dbg_m2_target = 0; dbg_m3_target = 0; dbg_m4_target = 0;
        Attitude_PID_History_Reset();
        target_yaw = current_euler.yaw;
        small_driver_set_duty(0, 0, 0, 0);
        return;
    }

    if (takeoff_abort_latched)
    {
        // 已经触发过起飞中止，保持锁死关电机。
        // 只有上面的 ALT_IDLE/低油门分支会清除 latch，避免翻倒扶正后自动再次起飞。
        dbg_takeoff_abort = 1;
        takeoff_elapsed_ms = 0;
        takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_trim_blend = 0.0f;

        p_out = 0.0f; r_out = 0.0f; y_out = 0.0f; y_limited = 0.0f;
        yaw_i_enable = 0;
        dbg_m1_target = 0; dbg_m2_target = 0; dbg_m3_target = 0; dbg_m4_target = 0;
        Attitude_PID_History_Reset();
        target_yaw = current_euler.yaw;
        small_driver_set_duty(0, 0, 0, 0);
        return;
    }

    if (takeoff_state != TAKEOFF_STATE_FULL)
    {
        takeoff_elapsed_ms += TAKEOFF_DT_MS;

        if (takeoff_elapsed_ms <= TAKEOFF_PRESPIN_MS)
        {
            // 阶段 1：同步预转。四个电机给完全相同的正向目标 duty，不叠加任何姿态修正。
            // 这个阶段飞机仍贴地，姿态 PID 控制不了机体，强行修正只会把某一边撬起来。
            int16 idle_duty = (int16)f_limit(MOTOR_IDLE_DUTY, MOTOR_MIN, MOTOR_MAX);

            takeoff_state = TAKEOFF_STATE_PRESPIN;
            dbg_takeoff_state = TAKEOFF_STATE_PRESPIN;
            dbg_trim_blend = 0.0f;

            pitch_target = pitch_zero_offset;
            roll_target = roll_zero_offset;
            p_target_rate = 0.0f;
            r_target_rate_dbg = 0.0f;
            y_target_rate_dbg = 0.0f;
            p_out = 0.0f; r_out = 0.0f; y_out = 0.0f; y_limited = 0.0f;
            yaw_i_enable = 0;
            dbg_pitch_rate_fb = p_rate_fb;
            dbg_roll_rate_fb = r_rate_fb;
            dbg_yaw_rate_fb = y_rate_fb;

            dbg_m1_target = idle_duty;
            dbg_m2_target = idle_duty;
            dbg_m3_target = idle_duty;
            dbg_m4_target = idle_duty;

            // 地面预转期间持续清积分，并把当前航向作为起飞后锁定航向。
            Attitude_PID_History_Reset();
            // 预转阶段虽然清积分，但保留当前角速度作为 D 项历史，减少进入 ramp 阶段瞬间冲击。
            pid_pitch_rate.last_current = p_rate_fb;
            pid_roll_rate.last_current  = r_rate_fb;
            pid_yaw_rate.last_current   = y_rate_fb;
            target_yaw = current_euler.yaw;

            small_driver_set_duty(idle_duty, idle_duty, idle_duty, idle_duty);  // 预转方向与正常飞行一致(全正=全部正转上推)，原(-,+,-,-)与正常飞行符号矛盾
            return;
        }
        else if (takeoff_elapsed_ms <= (TAKEOFF_PRESPIN_MS + TAKEOFF_RAMP_MS))
        {
            // 阶段 2：缓慢加油 + 限制姿态输出。
            // base_duty 从预转 duty 线性拉到外部给定油门，trim 也同步从 0 混到 1。
            float ramp_t = (float)(takeoff_elapsed_ms - TAKEOFF_PRESPIN_MS) / (float)TAKEOFF_RAMP_MS;
            ramp_t = f_limit(ramp_t, 0.0f, 1.0f);

            takeoff_state = TAKEOFF_STATE_RAMP_LIMIT;
            dbg_takeoff_state = TAKEOFF_STATE_RAMP_LIMIT;
            dbg_trim_blend = ramp_t;

            // 高度起飞油门在前 600ms 可能还没爬到预转 duty。
            // ramp 目标先钳到 MOTOR_IDLE_DUTY 以上，避免预转后电机反而短暂降速。
            float ramp_target = f_limit(base_duty, MOTOR_IDLE_DUTY, 9000.0f);
            base_duty = MOTOR_IDLE_DUTY + (ramp_target - MOTOR_IDLE_DUTY) * ramp_t;
        }
        else
        {
            // 阶段 3：离地后恢复完整控制。
            takeoff_state = TAKEOFF_STATE_FULL;
            dbg_takeoff_state = TAKEOFF_STATE_FULL;
            dbg_trim_blend = 1.0f;
        }
    }
    else
    {
        dbg_takeoff_state = TAKEOFF_STATE_FULL;
        dbg_trim_blend = 1.0f;
    }
    
    // 起飞 trim 高度渐入：
    // TRIM_ENABLE_HEIGHT_CM 以下尽量用自动零点水平起飞，不让飞行 trim 过早把机体带出去；
    // TRIM_ENABLE_HEIGHT_CM~TRIM_FULL_HEIGHT_CM 按高度逐渐加入 trim；之后恢复完整稳定段 trim。
    if (current_height_cm < TRIM_ENABLE_HEIGHT_CM)
    {
        takeoff_trim_weight = 0.0f;
    }
    else if (current_height_cm < TRIM_FULL_HEIGHT_CM)
    {
        takeoff_trim_weight = (current_height_cm - TRIM_ENABLE_HEIGHT_CM) /
                              (TRIM_FULL_HEIGHT_CM - TRIM_ENABLE_HEIGHT_CM);
    }
    else
    {
        takeoff_trim_weight = 1.0f;
    }
    takeoff_trim_weight = f_limit(takeoff_trim_weight, 0.0f, 1.0f);
    final_trim_blend = dbg_trim_blend * takeoff_trim_weight;
    dbg_takeoff_trim_weight = takeoff_trim_weight;

    pitch_target = pitch_zero_offset + pitch_flight_trim * final_trim_blend;
#if FLOW_HOLD_ENABLE
    pitch_target += upf_pitch_corr;
#endif
    p_target_rate = PID_Compute_Angle(&pid_pitch_angle, pitch_target, current_euler.pitch, 0.002f);
    
    // 二次保险限幅，防止后续调参时 out_limit 被改大又把机体打崩
    p_target_rate = f_limit(p_target_rate, -60.0f, 60.0f);
    p_out = PID_Compute_Rate(&pid_pitch_rate, p_target_rate, p_rate_fb, 0.002f);
    ladrc_observe_pitch(p_target_rate, p_rate_fb, 0.002f);  /* LADRC 旁观 */

    roll_target = roll_zero_offset
                + roll_flight_trim * final_trim_blend;
#if FLOW_HOLD_ENABLE
    roll_target += upf_roll_corr;  //加上光流修正
#endif
    r_target_rate_dbg = PID_Compute_Angle(&pid_roll_angle, roll_target, current_euler.roll, 0.002f);
    r_target_rate_dbg = f_limit(r_target_rate_dbg, -60.0f, 60.0f);
    r_out = PID_Compute_Rate(&pid_roll_rate, r_target_rate_dbg, r_rate_fb, 0.002f);
    ladrc_observe_roll(r_target_rate_dbg, r_rate_fb, 0.002f);  /* LADRC 旁观 */

#if YAW_HEADING_HOLD
    // 正常模式：航向外环 → 目标偏航角速度
    y_target_rate_dbg = PID_Compute_Angle(&pid_yaw_angle, target_yaw, current_euler.yaw, 0.002f);
    y_target_rate_dbg = f_limit(y_target_rate_dbg, -55.0f, 55.0f);
#else
    // 路A诊断：关掉航向外环，目标角速度恒为 0（不锁航向，只把转速拉回零）
    y_target_rate_dbg = 0.0f;
#endif

    // 起飞阶段 Yaw 容易被地效、瞬时反扭矩和姿态扰动带偏。
    // 低高度或垂直速度过大时只保留 P 控制和前馈，不让积分提前被推到某一侧。
   // if (current_height_cm < 50.0f || fabsf(current_speed_z) > 16.0f)
    if (current_height_cm < 50.0f)
    {
        yaw_i_enable = 0;
    }
    else
    {
        yaw_i_enable = 1;
    }

    /* yaw 角速度环：PID 控制 */
    y_out = PID_Compute_Yaw_Rate(&pid_yaw_rate, y_target_rate_dbg, y_rate_fb, 0.002f);
    ladrc_observe_yaw(y_target_rate_dbg, y_rate_fb, 0.002f);  /* LADRC 旁观 */

    /* ---- Yaw 故障检测（借鉴MaplePilot/无名飞控） ----
     * yaw PID输出大(>一半限幅)但实测yaw速率小(<30deg/s)持续2秒，
     * 则认为是外力卡住/饱和→复位I并重锁航向，防YI无限积累。 */
    {
        static uint16_t yaw_stall_cnt = 0;
        float yaw_out_half = pid_yaw_rate.out_limit * 0.5f;
        if (fabsf(y_out) > yaw_out_half && fabsf(y_rate_fb) < 30.0f)
        {
            yaw_stall_cnt++;
            if (yaw_stall_cnt > 790U)   // 1.58s @ 500Hz（折中：给I积累时间，又保留故障检测）
            {
                pid_yaw_rate.integral = 0.0f;
                target_yaw = current_euler.yaw;
                yaw_stall_cnt = 0;
            }
        }
        else
        {
            if (yaw_stall_cnt > 0) yaw_stall_cnt--;
        }
    }
    
    //仅角速度阻尼，不锁航向
//    y_target_rate_dbg = 0.0f;
//    y_out = PID_Compute_Yaw_Rate(&pid_yaw_rate, 0.0f, y_rate_fb, 0.002f);

    dbg_pitch_trim = pitch_zero_offset + pitch_flight_trim * final_trim_blend;
    dbg_roll_trim = roll_zero_offset + roll_flight_trim * final_trim_blend;
    dbg_pitch_rate_fb = p_rate_fb;
    dbg_roll_rate_fb = r_rate_fb;
    dbg_yaw_rate_fb = y_rate_fb;
    
    yaw_bias_adapt = Yaw_Bias_By_Throttle(base_duty);

    // Yaw 前馈保持全量参与：
    // 实测 0.75/0.95 渐入都会让起飞阶段 yaw 需要 PID 额外补救，稳定性不如完整前馈。
    // 因此只保留 pitch/roll trim 高度渐入，yaw 前馈不做起飞渐入。
    yaw_takeoff_weight = 1.0f;
#if YAW_FF_ENABLE
    yaw_bias_for_mix = yaw_bias_adapt * Battery_Comp_Get_Scale();
#else
    yaw_bias_for_mix = 0.0f;   // 诊断：关掉偏航前馈，纯靠 PID
#endif
    dbg_yaw_takeoff_weight = yaw_takeoff_weight;
    dbg_yaw_bias_for_mix = yaw_bias_for_mix;

    y_limited = f_limit(y_out + yaw_bias_for_mix, -520.0f, 520.0f); // Yaw PID + 全量前馈

    if (takeoff_state == TAKEOFF_STATE_RAMP_LIMIT)
    {
        // 起飞缓升阶段限制姿态修正权重，避免刚离地时某个电机被 PID 突然拉得过高或过低。
        p_out = f_limit(p_out, -TAKEOFF_P_OUT_LIMIT, TAKEOFF_P_OUT_LIMIT);
        r_out = f_limit(r_out, -TAKEOFF_R_OUT_LIMIT, TAKEOFF_R_OUT_LIMIT);
        y_limited = f_limit(y_limited, -TAKEOFF_Y_OUT_LIMIT, TAKEOFF_Y_OUT_LIMIT);
    }

    float y_mix = YAW_MIX_SIGN * y_limited;
    
    // --- 安全与状态流转 ---
    static uint8_t crash_cnt = 0;
    
    if (fabs(current_euler.pitch) > 70.0f || fabs(current_euler.roll) > 70.0f) 
    {
        // 坠机保护
      crash_cnt++;
      
        // 只有连续 35 次 (即 50 毫秒) 角度都大于 70 度，才认定为真坠机！
        if (crash_cnt > 35) 
        {
            small_driver_set_duty(0, 0, 0, 0); 
            dbg_m1_target = 0; dbg_m2_target = 0; dbg_m3_target = 0; dbg_m4_target = 0;
            takeoff_elapsed_ms = 0;
            takeoff_state = TAKEOFF_STATE_IDLE;
            dbg_takeoff_state = TAKEOFF_STATE_IDLE;
            dbg_trim_blend = 0.0f;
            Attitude_PID_History_Reset();
            target_yaw = current_euler.yaw;
        }
    } 
    else if (throttle < 400) 
    //else if (flight_state == ALT_IDLE || throttle <= THR_MOTOR_START + 50)
    { 
        crash_cnt = 0; // 只要姿态恢复正常或怠速，计数器清零
        // 怠速在地面的状态
        small_driver_set_duty (0, 0, 0, 0); 
        dbg_m1_target = 0; dbg_m2_target = 0; dbg_m3_target = 0; dbg_m4_target = 0;
        takeoff_elapsed_ms = 0;
        takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_takeoff_state = TAKEOFF_STATE_IDLE;
        dbg_trim_blend = 0.0f;
        Attitude_PID_History_Reset();
        
        // 【核心无遥控逻辑】：只要在地上还没起飞，就不断将当前朝向设为目标朝向。
        // 一旦油门推过 400 升空，这个 target_yaw 就不再更新，飞机将死死锁住这一刻的方向！
        target_yaw = current_euler.yaw; 
    } 
    else {
          crash_cnt = 0; // 正常飞行时，保护计数器清零
        // 升空正常飞行
        // 混控前：按动力余量缩放姿态输出，防止单个电机冲顶/掉底
        Attitude_Mix_Headroom_Limit(base_duty, &p_out, &r_out, &y_mix);

        // M1 (右前)：需要响应 Pitch的前部(+) 和 Roll的右侧(-)
        int16 m1_target = (int16)f_limit(base_duty + p_out + r_out + y_mix, MOTOR_MIN, MOTOR_MAX);

        // M2 (左前)：需要响应 Pitch的前部(+) 和 Roll的左侧(+)
        int16 m2_target = (int16)f_limit(base_duty + p_out - r_out - y_mix, MOTOR_MIN, MOTOR_MAX);

        // M3 (右后)：需要响应 Pitch的后部(-) 和 Roll的右侧(-)
        int16 m3_target = (int16)f_limit(base_duty - p_out + r_out - y_mix, MOTOR_MIN, MOTOR_MAX);

        // M4 (左后)：需要响应 Pitch的后部(-) 和 Roll的左侧(+)
        int16 m4_target = (int16)f_limit(base_duty - p_out - r_out + y_mix, MOTOR_MIN, MOTOR_MAX);

        dbg_m1_target = m1_target;
        dbg_m2_target = m2_target;
        dbg_m3_target = m3_target;
        dbg_m4_target = m4_target;

        small_driver_set_duty (m1_target, m2_target, m3_target, m4_target);
    }
}

/* ============================================================================
 * IMU 温漂陀螺零偏后台重校准
 *
 * 原理：IMU660RA 上电后前几分钟温漂最明显（可达 ±0.5°/s）。
 *       boot 时的 Int_MPU6050_calculate_offset 标定一次，但温漂会持续。
 *       本函数在飞机静止(低陀螺方差)且电机未运转(低油门)时，
 *       持续追踪陀螺残余偏置并缓慢更新 gyro_x/y/z_offset。
 *
 * 调用：Motor_Control_Mixing 内每 2ms 调一次
 * 条件：方差 < 阈值(≈静止) 且 throttle < 500(≈不上电) 持续 3s
 * 更新：每次吸收残余偏置的 1%（~100 次校准事件 = ~100s 静止时间达到稳态）
 * ============================================================================ */
#define GYRO_CAL_WINDOW_SAMPLES   (250)   // 250×2ms = 0.5s 统计窗口
#define GYRO_CAL_VAR_THRESH_LSB2  (15.0f) // 方差阈值(LSB²)，静止时通常 <5
#define GYRO_CAL_SETTLE_MS        (3000U) // 持续静止多久才更新(3s)
#define GYRO_CAL_LEAK_RATE        (0.01f) // 每次吸收 1% 残余，防跳变

void Gyro_Bias_Recalibrate_2ms(int16 gx, int16 gy, int16 gz, float throttle)
{
    static float  sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    static float  sum_sq_x = 0.0f, sum_sq_y = 0.0f, sum_sq_z = 0.0f;
    static uint16_t cnt = 0;
    static uint16_t stationary_ms = 0;

    /* 累计均值和二阶矩（用于方差 = E[X²] - E[X]²） */
    sum_x += (float)gx;
    sum_y += (float)gy;
    sum_z += (float)gz;
    sum_sq_x += (float)gx * (float)gx;
    sum_sq_y += (float)gy * (float)gy;
    sum_sq_z += (float)gz * (float)gz;
    cnt++;

    if (cnt < GYRO_CAL_WINDOW_SAMPLES) return;

    /* 窗口满 → 计算均值和方差 */
    float mean_x = sum_x / (float)GYRO_CAL_WINDOW_SAMPLES;
    float mean_y = sum_y / (float)GYRO_CAL_WINDOW_SAMPLES;
    float mean_z = sum_z / (float)GYRO_CAL_WINDOW_SAMPLES;
    float var_x  = sum_sq_x / (float)GYRO_CAL_WINDOW_SAMPLES - mean_x * mean_x;
    float var_y  = sum_sq_y / (float)GYRO_CAL_WINDOW_SAMPLES - mean_y * mean_y;
    float var_z  = sum_sq_z / (float)GYRO_CAL_WINDOW_SAMPLES - mean_z * mean_z;

    /* 复位累加器 */
    sum_x = sum_y = sum_z = 0.0f;
    sum_sq_x = sum_sq_y = sum_sq_z = 0.0f;
    cnt = 0;

    /* 判断是否静止：三轴方差低 + 油门低（电机不转） */
    if (var_x < GYRO_CAL_VAR_THRESH_LSB2 &&
        var_y < GYRO_CAL_VAR_THRESH_LSB2 &&
        var_z < GYRO_CAL_VAR_THRESH_LSB2 &&
        throttle < 500.0f)
    {
        stationary_ms += GYRO_CAL_WINDOW_SAMPLES * 2;  // ~500ms/窗口
        if (stationary_ms >= GYRO_CAL_SETTLE_MS)
        {
            /* last_gyro = raw - offset，所以其均值 = 残余偏置。
             * 将残余偏置缓慢吸收进 offset，消除温漂。
             * 用 += 而非 =，且只吸收一小部分，防止单次误检导致跳变。 */
            gyro_x_offset += (int32)(mean_x * GYRO_CAL_LEAK_RATE);
            gyro_y_offset += (int32)(mean_y * GYRO_CAL_LEAK_RATE);
            gyro_z_offset += (int32)(mean_z * GYRO_CAL_LEAK_RATE);
            stationary_ms = 0;
        }
    }
    else
    {
        stationary_ms = 0;
    }
}
