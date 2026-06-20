#include "dl1b_altitude.h"
#include "math.h"
#include "battery_comp.h"
#include "quaternion.h" // 姿态角 current_euler 在此头文件中声明

extern volatile uint32_t sys_time_ms;

Alt_State_e flight_state = ALT_IDLE;

PID_t pid_alt_pos;
PID_t pid_alt_vel;

// float target_height_cm = 140.0f; // 旧值：1.4m 目标，0.5m 初测时会让日志 Htar 混乱
float target_height_cm = 0.0f;
float current_height_cm = 0.0f;
float current_speed_z = 0.0f;
volatile float throttle_output = 0.0f;
float dbg_alt_err = 0.0f;
float dbg_target_vel = 0.0f;
float dbg_vel_err = 0.0f;
float dbg_vel_out = 0.0f;
float dbg_thr_base = 0.0f;
float dbg_thr_alt = 0.0f;
float target_speed_z = 0.0f;
float dbg_alt_pos_out = 0.0f;
float dbg_alt_vel_out = 0.0f;
float alt_out = 0.0f;
float dbg_thr_precomp = 0.0f;
float dbg_thr_after_tilt = 0.0f;
float dbg_thr_after_batt = 0.0f;
float dbg_batt_delta = 0.0f;

static float takeoff_throttle = THR_MOTOR_START;

/**
 * @brief 定高双环 PID 参数初始化
 */
void Altitude_System_Init(void)
{
    // 位置环：高度差 → 期望速度，只要 P
    pid_alt_pos.kp = 0.60f;  //0.55
    pid_alt_pos.ki = 0.0f;
    pid_alt_pos.kd = 0.0f;
    pid_alt_pos.out_limit = CLIMB_UP_MAX_SPEED;

    // 速度环：速度差 → 油门补偿，I 负责自动学习悬停油门
    // pid_alt_vel.kp = 5.2f;  // 旧值：刹车偏软
    // pid_alt_vel.ki = 0.24f; // 旧值：积分偏强，容易把悬停点学高
    pid_alt_vel.kp = 4.0f;        // 7.5→4.0：降Z轴速度环P
    pid_alt_vel.ki = 0.16f; 
    pid_alt_vel.kd = 0.0f;   // DL1B 单传感器严禁加 D
    // pid_alt_vel.i_limit  = 220.0f;  // 旧值
    // pid_alt_vel.out_limit = 500.0f; // 旧值
    pid_alt_vel.i_limit  = 120.0f;
    pid_alt_vel.out_limit = 500.0f;   // 原 600
}

/**
 * @brief DL1B 测距异常剔除 + 一阶低通滤波
 */
static float TOF_Data_Filter(float raw_dist_cm)
{
    static float window[3] = {0};
    static uint8_t init = 0;
    static uint8_t ptr  = 0;
    static float last_out = 0;

    if (!init) {
        window[0] = window[1] = window[2] = raw_dist_cm;
        last_out = raw_dist_cm;
        init = 1;
        return raw_dist_cm;
    }

    float mean = (window[0] + window[1] + window[2]) / 3.0f;
    // 偏离均值超 15cm 视为跳变噪声，用均值替代
    float valid_dist = (fabs(raw_dist_cm - mean) > 15.0f) ? mean : raw_dist_cm;

    window[ptr] = valid_dist;
    ptr = (ptr + 1) % 3;

    float out_dist = 0.3f * valid_dist + 0.7f * last_out;
    last_out = out_dist;
    return out_dist;
}

/**
 * @brief Z 轴速度估计器（差分 + 低通）
 * 【fix】系数从 0.05/0.95 改为 0.20/0.80，响应时间常数从 ~0.2s 缩短至 ~0.05s
 *        速度环阻尼得以及时生效，定高超调明显减少
 */
//static float Speed_Z_Estimator(float cur_height, float dt)
//{
//    static float last_h     = 0.0f;
//    static float last_speed = 0.0f;
//    static uint8_t init     = 0;
//
//    // 首帧只做初始化，避免启动时出现虚假速度尖峰
//    if (!init) {
//        last_h = cur_height;
//        last_speed = 0.0f;
//        init = 1;
//        return 0.0f;
//    }
//
//    float raw_speed = (cur_height - last_h) / dt;
//    raw_speed = f_limit(raw_speed, -80.0f, 80.0f);
//    last_h = cur_height;
//
//    float filtered_speed = 0.10f * raw_speed + 0.90f * last_speed;
//    last_speed = filtered_speed;
//    return filtered_speed;
//}

static float Speed_Z_Estimator(float cur_height, float dt, uint8_t tof_has_new)
{
    static float last_h     = 0.0f;
    static float last_speed = 0.0f;
    static uint8_t init     = 0;

    if (!init) {
        last_h = cur_height;
        last_speed = 0.0f;
        init = 1;
        return 0.0f;
    }

    float raw_speed = 0.0f;

    if (tof_has_new)
    {
        raw_speed = (cur_height - last_h) / dt;
        raw_speed = f_limit(raw_speed, -80.0f, 80.0f);

        last_h = cur_height;   // ✅ 只有新数据才更新
    }
    else
    {
        raw_speed = last_speed;  // 或者直接用上次速度
    }

    // float filtered_speed = 0.10f * raw_speed + 0.90f * last_speed;  // 旧值：TOF 20~30ms 一帧时时间常数 ~200ms，速度环阻尼滞后
    float filtered_speed = 0.25f * raw_speed + 0.75f * last_speed;
    last_speed = filtered_speed;

    return filtered_speed;
}

/**
 * @brief 倾角补偿：飞机前倾时自动补偿 Z 轴升力损失
 */
float Altitude_Tilt_Compensate(float base_thr, float pitch_deg, float roll_deg)
{
    float pitch_rad = pitch_deg * 0.01745329f;
    float roll_rad  = roll_deg  * 0.01745329f;

    float cos_p_cos_r = cosf(pitch_rad) * cosf(roll_rad);
    if (cos_p_cos_r < 0.5f) cos_p_cos_r = 0.5f; // 限制最大补偿约 60°

    if (base_thr <= THR_MOTOR_START) return base_thr;

    float compensated_thr = ((base_thr - THR_MOTOR_START) / cos_p_cos_r) + THR_MOTOR_START;
    return compensated_thr;
}

/**
 * @brief 【内部】串级定高 PID 计算核心，ALT_HOLD / ALT_LANDING 共用
 */
static void Altitude_PID_Compute(float dt, uint8_t tof_has_new)
{
    // 外环（位置环）：高度差 → 期望速度
    pid_alt_pos.error = target_height_cm - current_height_cm;
    dbg_alt_err = pid_alt_pos.error;

    float target_vel = pid_alt_pos.kp * pid_alt_pos.error;
    target_vel = f_limit(target_vel, CLIMB_DOWN_MAX_SPEED, CLIMB_UP_MAX_SPEED);
    dbg_target_vel = target_vel;
    target_speed_z = target_vel;
    dbg_alt_pos_out = target_vel;

    // 内环（速度环）：速度差 → 油门补偿
    pid_alt_vel.error = target_vel - current_speed_z;
    dbg_vel_err = pid_alt_vel.error;
    
    //积分泄放
    // pid_alt_vel.integral *= 0.9985f;  //0。995  0.998  // 旧值：泄放过快，悬停油门难收敛
    pid_alt_vel.integral *= 0.9998f;

    // 积分分离：仅在有新 TOF 数据且误差不大时才积分，防暴走
    if (tof_has_new && fabs(pid_alt_vel.error) < 80.0f) {
        pid_alt_vel.integral += pid_alt_vel.ki * pid_alt_vel.error * dt;
        pid_alt_vel.integral  = f_limit(pid_alt_vel.integral,
                                        -pid_alt_vel.i_limit,
                                         pid_alt_vel.i_limit);
    }

    float vel_pid_out = (pid_alt_vel.kp * pid_alt_vel.error) + pid_alt_vel.integral;
    vel_pid_out = f_limit(vel_pid_out, -pid_alt_vel.out_limit, pid_alt_vel.out_limit);
    dbg_vel_out = vel_pid_out;
    dbg_alt_vel_out = vel_pid_out;

    dbg_thr_base = THR_HOVER_DEFAULT;

    // 高度环真正输出的油门修正量：
    // 正数 = 加油门，负数 = 减油门。不要混入倾角/电压补偿，避免日志误判高度环方向。
    dbg_thr_alt = vel_pid_out;
    alt_out = vel_pid_out;

    float final_thr = THR_HOVER_DEFAULT + vel_pid_out;
    dbg_thr_precomp = final_thr;

    final_thr = Altitude_Tilt_Compensate(final_thr, current_euler.pitch, current_euler.roll);
    dbg_thr_after_tilt = final_thr;

    float final_thr_batt = Battery_Comp_Apply(final_thr);    // 新增：电压补偿
    dbg_thr_after_batt = final_thr_batt;
    dbg_batt_delta = final_thr_batt - final_thr;

    float min_thr = (flight_state == ALT_LANDING) ? THR_MOTOR_START : THR_MIN_OUTPUT;
    throttle_output = f_limit(final_thr_batt, min_thr, THR_MAX_OUTPUT);
}

/**
 * @brief 自动飞行控制核心状态机
 * 调用频率：主循环每 10ms 调用一次（dt ≈ 0.01s）
 */
void Altitude_Control_Task(float dt, uint8 tof_has_new)
{
    static float land_time_s = 0.0f;
    // static float hold_time_s = 0.0f;   // 调试期间自动降落已关闭，恢复时取消注释
    static float tof_lost_time_s = 0.0f;
    static float takeoff_tof_grace_s = 0.0f;
    static float tof_dt_acc = 0.0f;
    float takeoff_thr_batt = 0.0f;
    float i_temp = 0.0f;

    // 定高任务是 10ms 调一次，但 DL1B 不一定每次都有新帧。
    // 速度估计必须使用“TOF 新帧间隔”，否则 20/30ms 一帧时 Vz 会被放大。
    tof_dt_acc += dt;

    // ── 1. 数据采集 ──────────────────────────────────────────────
    if (tof_has_new) {
        float raw_cm = (float)dl1b_distance_mm / 10.0f;
        if (raw_cm > 800.0f || raw_cm < 0.0f) raw_cm = current_height_cm;
        current_height_cm = TOF_Data_Filter(raw_cm);
        float tof_dt = f_limit(tof_dt_acc, 0.01f, 0.08f);
        current_speed_z = Speed_Z_Estimator(current_height_cm, tof_dt, 1);
        tof_dt_acc = 0.0f;
        tof_lost_time_s = 0.0f;
    } else {
        // 无新数据时速度衰减，避免旧值一直驱动速度环
        current_speed_z *= 0.98f;
        if (fabs(current_speed_z) < 0.1f) current_speed_z = 0.0f;
        tof_lost_time_s += dt;
    }

    // ── 2. 状态机 ────────────────────────────────────────────────
    // 【fix】原来 ALT_HOLD 缺少 break 导致穿透到 ALT_LANDING，
    //        现在四个 case 完全独立，共用逻辑提取到 Altitude_PID_Compute()
    switch (flight_state)
    {
        // ── 怠速：电机停转，积分清零 ──────────────────────────────
        case ALT_IDLE:
            // hold_time_s = 0.0f;
            land_time_s = 0.0f;
            takeoff_tof_grace_s = 0.0f;
            throttle_output  = 0;
            dbg_thr_base = 0.0f;
            dbg_thr_alt = 0.0f;
            alt_out = 0.0f;
            target_speed_z = 0.0f;
            dbg_alt_pos_out = 0.0f;
            dbg_alt_vel_out = 0.0f;
            dbg_thr_precomp = 0.0f;
            dbg_thr_after_tilt = 0.0f;
            dbg_thr_after_batt = 0.0f;
            dbg_batt_delta = 0.0f;
            target_height_cm = 0.0f;     // 每次重新起飞前清空目标高度，避免沿用上一次 Htar
            takeoff_throttle = THR_MOTOR_START;
            pid_alt_vel.integral = 0;
            break;

        // ── 起飞：电机预转 → Vz速度PID恒定爬升 → 到目标附近切定高 ───
        case ALT_TAKEOFF:
            land_time_s = 0.0f;

            // TOF 长时间丢失 → 降落保护
            if (tof_lost_time_s > ALT_TOF_LOST_FAILSAFE_S) {
                takeoff_tof_grace_s += dt;
                takeoff_thr_batt = Battery_Comp_Apply(takeoff_throttle);
                throttle_output = f_limit(takeoff_thr_batt, THR_MOTOR_START, THR_MAX_OUTPUT);
                dbg_thr_base = takeoff_throttle;
                dbg_thr_alt = 0.0f; alt_out = 0.0f;
                target_speed_z = 0.0f;
                dbg_alt_pos_out = 0.0f; dbg_alt_vel_out = 0.0f;
                dbg_thr_precomp = takeoff_throttle;
                dbg_thr_after_tilt = takeoff_throttle;
                dbg_thr_after_batt = takeoff_thr_batt;
                dbg_batt_delta = takeoff_thr_batt - takeoff_throttle;
                if (takeoff_tof_grace_s > ALT_TAKEOFF_TOF_GRACE_S) {
                    flight_state = ALT_IDLE;
                }
                break;
            }
            takeoff_tof_grace_s = 0.0f;

            // 第一段：电机预转，快速越过死区
            if (takeoff_throttle < 3800.0f) {
                takeoff_throttle += 2500.0f * dt;
                takeoff_thr_batt = Battery_Comp_Apply(takeoff_throttle);
                throttle_output = f_limit(takeoff_thr_batt, THR_MOTOR_START, THR_MAX_OUTPUT);
                dbg_thr_base = takeoff_throttle;
                dbg_thr_alt = 0.0f; alt_out = 0.0f;
                target_speed_z = 0.0f;
                dbg_alt_pos_out = 0.0f; dbg_alt_vel_out = 0.0f;
                dbg_thr_precomp = takeoff_throttle;
                dbg_thr_after_tilt = takeoff_throttle;
                dbg_thr_after_batt = takeoff_thr_batt;
                dbg_batt_delta = takeoff_thr_batt - takeoff_throttle;
                break;
            }

            // 第二段：Vz速度PID恒定爬升，位置环目标设120cm
            // 位置环输出被CLIMB_UP_MAX_SPEED限幅，等效恒定15cm/s爬升
            target_height_cm = ALT_HOLD_TARGET_CM;
            Altitude_PID_Compute(dt, tof_has_new);

            // 到达目标高度附近 → 切定高模式（位置环+速度环）
            if (tof_has_new && current_height_cm > ALT_HOLD_TARGET_CM - 10.0f)
            {
                flight_state = ALT_HOLD;
                // 速度环积分已由Altitude_PID_Compute持续更新，不需要预填
                takeoff_tof_grace_s = 0.0f;
            }
            break;

        // ── 悬停：目标高度缓爬至 ALT_HOLD_TARGET_CM 后锁定 ────────
        case ALT_HOLD:
            land_time_s = 0.0f;
            takeoff_tof_grace_s = 0.0f;

            // 定高阶段 TOF 长时间丢失，转降落保护
//            if (tof_lost_time_s > ALT_TOF_LOST_FAILSAFE_S) {
//                flight_state = ALT_LANDING;
//                // hold_time_s = 0.0f;
//                break;
//            }

            if (target_height_cm < ALT_HOLD_TARGET_CM) {
                // target_height_cm += 16.0f * dt;         // 旧值：目标爬升偏快
                target_height_cm += 12.0f * dt;            // C1：目标高度爬升放缓(18→12)，配合位置环在爬升段就生效
            } else {
                target_height_cm = ALT_HOLD_TARGET_CM;
            }

            Altitude_PID_Compute(dt, tof_has_new);

            // 【调试期间临时关闭自动降落】定高调试中，不希望 10s 后自动进入 ALT_LANDING
            // 到达目标高度附近后按 dt 累计计时，连续保持 10s 后自动降落
            // if (fabs(current_height_cm - ALT_HOLD_TARGET_CM) <= ALT_HOLD_BAND_CM) {
            //     hold_time_s += dt;
            //     if (hold_time_s >= ALT_HOLD_TIME_S) {
            //         flight_state = ALT_LANDING;
            //         hold_time_s = 0.0f;
            //     }
            // } else {
            //     hold_time_s = 0.0f;
            // }
            // hold_time_s = 0.0f;   // 防止后面其它逻辑读到累计值
            break;

        // ── 降落：目标高度匀速下降，触地后断电 ───────────────────
        case ALT_LANDING:
            // hold_time_s = 0.0f;
            takeoff_tof_grace_s = 0.0f;
            target_height_cm -= 15.0f * dt;
            if (target_height_cm < 0.0f) target_height_cm = 0.0f;

            // 仅在 TOF 有效且真实高度足够低时确认触地，避免空中提前断电
            if (tof_has_new && current_height_cm < ALT_LAND_DETECT_CM) {
                land_time_s += dt;
                if (land_time_s >= ALT_LAND_CONFIRM_S) {
                    flight_state = ALT_IDLE;
                    land_time_s = 0.0f;

                    // 同一周期内已经确认触地，立即断油退出，不再继续计算一次高度 PID。
                    throttle_output = 0.0f;
                    pid_alt_vel.integral = 0.0f;
                    dbg_thr_base = 0.0f;
                    dbg_thr_alt = 0.0f;
                    alt_out = 0.0f;
                    target_speed_z = 0.0f;
                    dbg_alt_pos_out = 0.0f;
                    dbg_alt_vel_out = 0.0f;
                    dbg_thr_precomp = 0.0f;
                    dbg_thr_after_tilt = 0.0f;
                    dbg_thr_after_batt = 0.0f;
                    dbg_batt_delta = 0.0f;
                    break;
                }
            } else {
                land_time_s = 0.0f;
            }
            Altitude_PID_Compute(dt, tof_has_new);
            break;

        // ── 兜底：不认识的状态直接断电 ───────────────────────────
        default:
            // hold_time_s = 0.0f;
            land_time_s = 0.0f;
            takeoff_tof_grace_s = 0.0f;
            throttle_output = 0;
            dbg_thr_base = 0.0f;
            dbg_thr_alt = 0.0f;
            break;
    }
}
