#include "UP_FLOW_302.h"
#include "cam_share.h"
#include "mpu660ra.h"       /* filtered_data, current_euler */

/* 垂直速度——用于爬升检测，关IMU融合防重力分离正反馈振荡 */
extern float current_speed_z;
extern float current_height_cm;
#if 1   /* 与头文件 enable 开关保持一致 */
#include "math.h"

/*
 * ==================== 文件职责 ====================
 * 应用层：读取逐飞官方 zf_device_upflow302 解析好的帧，
 *        换算成物理速度（cm/s），LPF 滤波，PI 阻尼输出 pitch/roll 修正角。
 *
 * 底层（不在本文件）：
 *   - UART6 接收中断 → upflow302_receive_callback() (libraries/zf_device/)
 *   - XOR 校验、帧组装、写入 upflow302_receive 结构体
 *
 * 调用时序（主循环）：
 *   Up_Flow_302_Update(current_height_cm)
 *     ① 高度门控
 *     ② 检测新帧：upflow302_finsh_flag 置 1 表示底层刚收到一帧
 *     ③ 物理速度换算：v = pixel_integral × 100 × height_cm / timespan_us
 *     ④ 跳变剔除 + 一阶 LPF → upf_vel_x, upf_vel_y
 *     ⑤ valid 位/帧超时 → 决定 upf_data_valid
 *   Up_Flow_302_Speed_Damp(dt)
 *     ⑥ PI(0 - v) → pitch_corr / roll_corr
 */

/* ==================== 对外可见变量 ==================== */
float  upf_vel_x = 0.0f;
float  upf_vel_y = 0.0f;
uint8  upf_data_valid = 0;
uint8  upf_data_fresh = 0;
float  upf_dbg_raw_vx = 0.0f;
float  upf_dbg_raw_vy = 0.0f;
uint16 upf_dbg_frame_cnt = 0;
uint32 upf_dbg_last_frame_ms = 0;
float  upf_dbg_omega_x = 0.0f;
float  upf_dbg_omega_y = 0.0f;
float  upf_dbg_gyro_lpf_x = 0.0f;
float  upf_dbg_gyro_lpf_y = 0.0f;
float  upf_dbg_gyro_lpf_z = 0.0f;
float  upf_pitch_corr = 0.0f;
float  upf_roll_corr = 0.0f;
float  upf_dbg_err_vx = 0.0f;        /* 速度误差 X */
float  upf_dbg_err_vy = 0.0f;        /* 速度误差 Y */
float  upf_dbg_raw_out_pitch = 0.0f; /* 翻号前原始 pitch 输出 */
float  upf_dbg_raw_out_roll  = 0.0f; /* 翻号前原始 roll 输出 */

/* 速度环设定值(cm/s)：阶段1由 Cam_Vel_Mock_Update 写，将来由相机外环写。
 * 默认 0 = 悬停 hold（和原版行为一致）。Speed_Damp 读它当作目标速度。*/
float  upf_target_vx = 0.0f;
float  upf_target_vy = 0.0f;

/* 第3步接口：车相对飞机坐标(cm)。mock 或真相机写，跟随外环读。*/
float  cam_rel_x = 0.0f;
float  cam_rel_y = 0.0f;
uint8  cam_valid = 0;
uint32 cam_last_update_ms = 0;   /* 视觉最近一次写入时刻(ms) */

/* 调试镜像 */
int16  cam_dbg_u = 0, cam_dbg_v = 0;
int16  cam_dbg_area = 0;
uint8  cam_dbg_maxg = 0;
float  cam_dbg_x = 0.0f, cam_dbg_y = 0.0f;
uint8  cam_dbg_valid = 0;
uint16 cam_dbg_rx_cnt = 0;
uint32 cam_dbg_last_raw = 0;
float  cam_dbg_pcomp = 0.0f;     /* 俯仰补偿像素 */
float  cam_dbg_rcomp = 0.0f;     /* 横滚补偿像素 */

extern volatile uint32_t sys_time_ms;     /* main 维护的毫秒时间戳 */

/* 视觉写入车相对坐标（由 Cam_IPC_Process 调，或外部UART调）*/
void Cam_Set_Target(float rel_x_cm, float rel_y_cm, uint8 valid)
{
    cam_rel_x = rel_x_cm;
    cam_rel_y = rel_y_cm;
    cam_valid = valid;
    cam_dbg_x = rel_x_cm;
    cam_dbg_y = rel_y_cm;
    cam_dbg_valid = valid;
    cam_last_update_ms = sys_time_ms;
}

/* ==================== CM7_1视觉 → CM7_0飞控 共享内存接收 ==================== */
static volatile int16  cam_px_u = 0;
static volatile int16  cam_px_v = 0;
static volatile uint8  cam_px_valid_raw = 0;
static volatile uint32 cam_px_recv_ms = 0;

void Cam_IPC_Init(void) { /* CM7_1预留信箱，CM7_0只读 */ }

void Cam_IPC_Process(float height_cm)
{
    static uint32 last_seq = 0;
    uint32 s = CAM_SHARE->seq;
    if (s != last_seq)
    {
        last_seq = s;
        int16 u = CAM_SHARE->px_u;
        int16 v = CAM_SHARE->px_v;
        uint8 vld = CAM_SHARE->valid;
        cam_dbg_rx_cnt++;
        cam_dbg_area = CAM_SHARE->area;
        cam_dbg_maxg = CAM_SHARE->maxg;
        cam_dbg_last_raw = ((uint32)(uint16)u << 16) | (uint16)v;
        if (vld) { cam_px_u = u; cam_px_v = v; cam_dbg_u = u; cam_dbg_v = v; cam_px_valid_raw = 1; }
        else     { cam_px_valid_raw = 0; }
        cam_px_recv_ms = sys_time_ms;
    }

    uint8 fresh = cam_px_valid_raw &&
                  ((uint32)(sys_time_ms - cam_px_recv_ms) < CAM_TIMEOUT_MS);

    if (fresh)
    {
        int16 px_u = cam_px_u;
        int16 px_v = cam_px_v;
        /* 仰角+横滚补偿：减掉飞机倾斜导致的像素偏移，防止位置环追假偏移
         * 补偿公式：comp = raw - FOCAL_PX * tan(angle)
         * 飞机低头(pitch负)→目标在画面中上移(v更负)→减掉tan(pitch)把目标拉回中心 */
        float pitch_rad = current_euler.pitch * DEG2RAD;
        float roll_rad  = current_euler.roll  * DEG2RAD;
        float tilt_u = CAM_FOCAL_PX * tanf(roll_rad);
        float tilt_v = CAM_FOCAL_PX * tanf(pitch_rad);
        cam_dbg_rcomp = tilt_u;
        cam_dbg_pcomp = tilt_v;
        /* 实测数据验证：右倾(ROL+)时u增大(目标偏右)，减tilt_u使u回中。
         * 抬头(PIT+)时v增大(目标偏下)，减tilt_v使v回中。
         * 两轴都用减号，数据验证有效。 */
        float comp_u = (float)px_u - tilt_u;
        float comp_v = (float)px_v - tilt_v;
    #if CAM_SWAP_UV
        float px_fwd = comp_u;
        float px_right = comp_v;
    #else
        float px_fwd = comp_v;
        float px_right = comp_u;
    #endif
        float cam_range_cm = height_cm - CAM_TARGET_HEIGHT_CM;
        if (cam_range_cm < CAM_MIN_RANGE_CM) cam_range_cm = CAM_MIN_RANGE_CM;
        float fwd_cm   = CAM_FWD_SIGN   * px_fwd   * cam_range_cm / CAM_FOCAL_PX;
        float right_cm = CAM_RIGHT_SIGN * px_right * cam_range_cm / CAM_FOCAL_PX;
        Cam_Set_Target(fwd_cm, right_cm, 1);
    }
    else
    {
        Cam_Set_Target(0.0f, 0.0f, 0);
    }
}

PID_t  pid_upf_vx;
PID_t  pid_upf_vy;

/* ==================== IMU融合状态 ==================== */
float  upf_imu_vx = 0.0f;           /* IMU加速度积分速度X(cm/s) */
float  upf_imu_vy = 0.0f;           /* IMU加速度积分速度Y(cm/s) */
float  upf_fused_vx = 0.0f;         /* 互补融合后速度X(cm/s) */
float  upf_fused_vy = 0.0f;         /* 互补融合后速度Y(cm/s) */
#if UPF_USE_IMU_FUSION
static float upf_alpha_effective = UPF_FUSION_ALPHA;  /* 当前有效融合系数，受爬升检测影响 */
void Up_Flow_302_IMU_Fusion(float dt)
{
    if (dt <= 0.0f) return;

    /* 1. 读取体坐标系加速度(原始LSB)并转为cm/s² */
    float ax_raw = (float)filtered_data.accel.imu660ra_acc_x;
    float ay_raw = (float)filtered_data.accel.imu660ra_acc_y; /* 左正 */
    float az_raw = (float)filtered_data.accel.imu660ra_acc_z; /* 上正 */

    float ax = ax_raw * UPF_ACC_LSB_TO_CMS2;  /* 前正 */
    float ay = ay_raw * UPF_ACC_LSB_TO_CMS2;  /* 左正 */
    float az = az_raw * UPF_ACC_LSB_TO_CMS2;  /* 上正 */

    /* 2. 用欧拉角计算重力在体坐标系的分量 */
    float sp = sinf(current_euler.pitch * DEG2RAD);  /* 抬头→正 */
    float cp = cosf(current_euler.pitch * DEG2RAD);
    float sr = sinf(current_euler.roll * DEG2RAD);   /* 右倾→正 */
    float cr = cosf(current_euler.roll * DEG2RAD);

    float gx = sp * UPF_G_CMS2;           /* 重力在body X分量 */
    float gy = -cp * sr * UPF_G_CMS2;     /* 重力在body Y分量(左正) */

    /* 3. 去重力得到水平加速度 */
    float acc_fwd = ax - gx;              /* 前进加速度(前正) */
    float acc_right = -(ay - gy);         /* y左正→转右正 */

    /* 4. 积分得IMU速度 + 一阶泄放(防漂移) */
    upf_imu_vx += acc_fwd * dt;
    upf_imu_vy += acc_right * dt;

    /* 泄放系数：0.990@10ms → 约1s衰减到37%，防漂移 */
    const float LEAK = 0.990f;
    upf_imu_vx *= LEAK;
    upf_imu_vy *= LEAK;

    /* 5. 限幅(防积爆) */
    if (upf_imu_vx > 100.0f) upf_imu_vx = 100.0f;
    if (upf_imu_vx < -100.0f) upf_imu_vx = -100.0f;
    if (upf_imu_vy > 100.0f) upf_imu_vy = 100.0f;
    if (upf_imu_vy < -100.0f) upf_imu_vy = -100.0f;

    /* 6. 爬升检测：Vz过大时重力分离不可靠→关IMU融合，防正反馈振荡 */
    uint8_t is_climbing = (fabsf(current_speed_z) > UPF_CLIMB_VZ_THRESHOLD) ? 1U : 0U;
    if (is_climbing) {
        /* 爬升中→瞬间切到目标α，IMU污染不进场 */
        upf_alpha_effective = UPF_CLIMB_ALPHA;
    } else {
        /* 悬停→平滑恢复融合，避免阶跃跳变 */
        upf_alpha_effective += UPF_ALPHA_SMOOTH * (UPF_FUSION_ALPHA - upf_alpha_effective);
        if (fabsf(upf_alpha_effective - UPF_FUSION_ALPHA) < 0.001f) {
            upf_alpha_effective = UPF_FUSION_ALPHA;
        }
    }

    /* 7. 互补滤波：α×(IMU÷scale) + (1-α)×光流
     * IMU 积分速度量级约 4×光流，先缩放匹配再融合 */
    upf_fused_vx = upf_alpha_effective * (upf_imu_vx * UPF_IMU_VEL_SCALE) + (1.0f - upf_alpha_effective) * upf_vel_x;
    upf_fused_vy = upf_alpha_effective * (upf_imu_vy * UPF_IMU_VEL_SCALE) + (1.0f - upf_alpha_effective) * upf_vel_y;
}
#endif /* UPF_USE_IMU_FUSION */

/* ==================== 角度域旋转补偿（V2）：陀螺仪角累积 + 环形缓冲 ====================
 * 用 2ms PIT 中断累积陀螺仪角位移，主循环做快照，光流新帧到达时查缓冲得到
 * "过去 timespan_us 内陀螺仪总转角"——与光流在同一积分窗口内测量，精确对齐。 */
#if UPF_USE_ANGLE_DOMAIN_COMP
/* ---- 2ms累积器（由 PIT ISR 调 Up_Flow_302_Gyro_Accum_2ms 更新） ---- */
static float upf_gyro_angle_x = 0.0f;   /* 累积 pitch 角位移 (rad)：由 gyro_y 积分，用于补偿 X 流 */
static float upf_gyro_angle_y = 0.0f;   /* 累积 roll  角位移 (rad)：由 gyro_x 积分，用于补偿 Y 流 */

/* ---- 10ms快照环形缓冲 ---- */
typedef struct {
    uint32_t time_ms;       /* 快照时刻 (sys_time_ms) */
    float    pitch_angle;   /* 当时累积 pitch 角位移 (rad) */
    float    roll_angle;    /* 当时累积 roll  角位移 (rad) */
} gyro_buf_entry_t;

static gyro_buf_entry_t upf_gyro_buf[UPF_GYRO_BUF_SIZE];
static uint8_t upf_gyro_buf_wr = 0;     /* 当前写入位置 */
static uint8_t upf_gyro_buf_cnt = 0;    /* 有效条目数 (0~UPF_GYRO_BUF_SIZE) */

/* 查缓冲：线性插值得到时刻 t_ms 的累积 pitch 角位移 */
static float gyro_buf_get_pitch_at(uint32_t t_ms)
{
    if (upf_gyro_buf_cnt == 0) return 0.0f;
    int8_t cnt = (int8_t)upf_gyro_buf_cnt;
    for (int8_t i = 0; i < cnt - 1; i++)
    {
        uint8_t curr = (uint8_t)((upf_gyro_buf_wr - i + UPF_GYRO_BUF_SIZE) % UPF_GYRO_BUF_SIZE);
        uint8_t prev = (uint8_t)((upf_gyro_buf_wr - i - 1 + UPF_GYRO_BUF_SIZE) % UPF_GYRO_BUF_SIZE);
        uint32_t tc = upf_gyro_buf[curr].time_ms;
        uint32_t tp = upf_gyro_buf[prev].time_ms;
        if (t_ms >= tp && t_ms <= tc)
        {
            if (tc == tp) return upf_gyro_buf[curr].pitch_angle;
            float frac = (float)(t_ms - tp) / (float)(tc - tp);
            return upf_gyro_buf[prev].pitch_angle + frac * (upf_gyro_buf[curr].pitch_angle - upf_gyro_buf[prev].pitch_angle);
        }
    }
    /* 超出缓冲范围 → 返回最老条目（短时外推误差可忽略） */
    uint8_t oldest = (uint8_t)((upf_gyro_buf_wr - cnt + 1 + UPF_GYRO_BUF_SIZE) % UPF_GYRO_BUF_SIZE);
    return upf_gyro_buf[oldest].pitch_angle;
}
static float gyro_buf_get_roll_at(uint32_t t_ms)
{
    if (upf_gyro_buf_cnt == 0) return 0.0f;
    int8_t cnt = (int8_t)upf_gyro_buf_cnt;
    for (int8_t i = 0; i < cnt - 1; i++)
    {
        uint8_t curr = (uint8_t)((upf_gyro_buf_wr - i + UPF_GYRO_BUF_SIZE) % UPF_GYRO_BUF_SIZE);
        uint8_t prev = (uint8_t)((upf_gyro_buf_wr - i - 1 + UPF_GYRO_BUF_SIZE) % UPF_GYRO_BUF_SIZE);
        uint32_t tc = upf_gyro_buf[curr].time_ms;
        uint32_t tp = upf_gyro_buf[prev].time_ms;
        if (t_ms >= tp && t_ms <= tc)
        {
            if (tc == tp) return upf_gyro_buf[curr].roll_angle;
            float frac = (float)(t_ms - tp) / (float)(tc - tp);
            return upf_gyro_buf[prev].roll_angle + frac * (upf_gyro_buf[curr].roll_angle - upf_gyro_buf[prev].roll_angle);
        }
    }
    uint8_t oldest = (uint8_t)((upf_gyro_buf_wr - cnt + 1 + UPF_GYRO_BUF_SIZE) % UPF_GYRO_BUF_SIZE);
    return upf_gyro_buf[oldest].roll_angle;
}

#endif /* UPF_USE_ANGLE_DOMAIN_COMP */

/* ==================== 内部状态 ==================== */
static float upf_raw_vel_x = 0.0f;
static float upf_raw_vel_y = 0.0f;
static float upf_lpf_vel_x = 0.0f;
static float upf_lpf_vel_y = 0.0f;
static uint8 upf_invalid_streak = 0;
static uint32 upf_last_valid_ms = 0;
static float cam_mock_t = 0.0f;   /* 速度指令 mock 的内部计时(秒)，Reset 时清零 */
/* 第3步 虚拟车位置 mock 的内部状态 */
static float cam_drone_x = 0.0f;  /* 飞机世界位置 X(积分光流,cm) */
static float cam_drone_y = 0.0f;  /* 飞机世界位置 Y(积分光流,cm) */
#if VCAR_MODE == 1
static float cam_vcar_t  = 0.0f;  /* 虚拟车轨迹计时(秒)，仅正弦模式用 */
#endif

/* 上一帧的 valid 状态。文件级 static 才能跨调用保持，并被 Reset() 清零。
 * 用途：当本轮没有新帧时，upf_data_valid 应该沿用上一帧的 valid 而不是
 * 因为 had_new_frame=0 就被错误地置 1。这是 Sophia 发现的关键 bug。 */
static uint8 upf_last_frame_valid = 0;

static inline void Up_Flow_302_Update_Fresh_Flag(void)
{
    upf_data_fresh = ((uint32)(sys_time_ms - upf_last_valid_ms) < UPF_VALID_HOLD_MS) ? 1U : 0U;
}

#if UPF_USE_ROTATION_COMP && !UPF_USE_ANGLE_DOMAIN_COMP
/* V1(旧)：陀螺仪 LPF 状态（rad/s）。
 * 注意：这里用了 PDF demo 的 LPF_1_ 公式，截止频率 8Hz */
static float upf_gyro_lpf_x = 0.0f;
static float upf_gyro_lpf_y = 0.0f;
static float upf_gyro_lpf_z = 0.0f;

/* LPF_1_ 等价于 PDF demo 里的一阶低通：α = 1 / (1 + 1/(hz × π × dt)) */
static inline float upf_lpf1(float out, float in, float hz, float dt)
{
    const float denom = (hz * 3.14159265f * dt);
    if (denom <= 0.0f) return out;
    const float alpha = 1.0f / (1.0f + 1.0f / denom);
    return out + alpha * (in - out);
}
#elif UPF_USE_ROTATION_COMP && UPF_USE_ANGLE_DOMAIN_COMP
/* V2(新)：角度域补偿，不使用 LPF，改用 2ms 累积 + 环形缓冲查询 */
#endif /* UPF_USE_ROTATION_COMP / ANGLE_DOMAIN_COMP */

/* 2ms PIT 中断调用：累积陀螺仪角位移（V2角度域补偿用）
 * 放在 #if UPF_USE_ROTATION_COMP 外面——函数体为空时编译器会优化掉调用 */
void Up_Flow_302_Gyro_Accum_2ms(void)
{
#if UPF_USE_ANGLE_DOMAIN_COMP
    /* last_gyro 在主 ISR 里已更新到最新 */
    const float gy = (float)last_gyro.imu660ra_gyro_y * UPF_GYRO_LSB_TO_RAD_S;  /* pitch rate (rad/s) */
    const float gx = (float)last_gyro.imu660ra_gyro_x * UPF_GYRO_LSB_TO_RAD_S;  /* roll rate (rad/s) */
    const float dt = 0.002f;

    /* X 流(前后)补偿用 pitch 角位移 = ∫pitch_rate dt = ∫gyro_y dt
     * Y 流(左右)补偿用 roll  角位移 = ∫roll_rate  dt = ∫gyro_x dt */
    upf_gyro_angle_x += gy * dt;
    upf_gyro_angle_y += gx * dt;

    /* 每 2ms 快照一次累积角到环形缓冲（比原来10ms快照精度高5倍）。
     * 快照放在累加之后，确保当前条目的角度已经包含了最新的2ms转动。 */
    upf_gyro_buf_wr = (uint8_t)((upf_gyro_buf_wr + 1U) % UPF_GYRO_BUF_SIZE);
    upf_gyro_buf[upf_gyro_buf_wr].time_ms     = sys_time_ms;
    upf_gyro_buf[upf_gyro_buf_wr].pitch_angle  = upf_gyro_angle_x;
    upf_gyro_buf[upf_gyro_buf_wr].roll_angle   = upf_gyro_angle_y;
    if (upf_gyro_buf_cnt < UPF_GYRO_BUF_SIZE) upf_gyro_buf_cnt++;
#else
    /* V1 模式或补偿关闭：什么都不做 */
#endif
}


/* ==================== Up_Flow_302_Init ==================== */
void Up_Flow_302_Init(void)
{
    /* ---- UART + 接收中断 ----
     * 调用 vendor 库的 upflow302_receive_init()，按库里宏配置 UART 号和引脚。
     * 库已经手动改成 UART_6 + P03.0/P03.1（GBK 编码保留）。
     * 注意：cm7_0_isr.c 的 uart6_isr 必须调用 upflow302_receive_callback()，
     *      否则收不到数据 */
    upflow302_receive_init();

    /* ---- 库内共享标志清零 ---- */
    upflow302_finsh_flag = 0;
    upflow302_state_flag = 1;

    /* ---- 应用层状态清零 ---- */
    upf_vel_x = 0.0f;
    upf_vel_y = 0.0f;
    upf_raw_vel_x = 0.0f;
    upf_raw_vel_y = 0.0f;
    upf_lpf_vel_x = 0.0f;
    upf_lpf_vel_y = 0.0f;
    upf_data_valid = 0;
    upf_data_fresh = 0;
    upf_last_valid_ms = sys_time_ms - UPF_VALID_HOLD_MS;
    upf_dbg_raw_vx = 0.0f;
    upf_dbg_raw_vy = 0.0f;
    upf_dbg_frame_cnt = 0;
    upf_dbg_last_frame_ms = 0;
    upf_dbg_omega_x = 0.0f;
    upf_dbg_omega_y = 0.0f;
    upf_dbg_gyro_lpf_x = 0.0f;
    upf_dbg_gyro_lpf_y = 0.0f;
    upf_dbg_gyro_lpf_z = 0.0f;
    upf_pitch_corr = 0.0f;
    upf_roll_corr = 0.0f;
#if UPF_USE_ROTATION_COMP && !UPF_USE_ANGLE_DOMAIN_COMP
    upf_gyro_lpf_x = 0.0f;
    upf_gyro_lpf_y = 0.0f;
    upf_gyro_lpf_z = 0.0f;
#endif
#if UPF_USE_ANGLE_DOMAIN_COMP
    upf_gyro_angle_x = 0.0f;
    upf_gyro_angle_y = 0.0f;
    upf_gyro_buf_wr = 0;
    upf_gyro_buf_cnt = 0;
    for (uint8_t i = 0; i < UPF_GYRO_BUF_SIZE; i++) {
        upf_gyro_buf[i].time_ms = sys_time_ms;
        upf_gyro_buf[i].pitch_angle = 0.0f;
        upf_gyro_buf[i].roll_angle  = 0.0f;
    }
#endif
    upf_last_frame_valid = 0;
    upf_invalid_streak = 0;

    /* ---- X 轴速度阻尼 PI ---- */
    pid_upf_vx.kp = FLOW_VX_KP;
    pid_upf_vx.ki = FLOW_VX_KI;
    pid_upf_vx.kd = FLOW_VX_KD;
    pid_upf_vx.i_limit = 8.0f;     // 10→20，给积分足够空间抗恒力
    pid_upf_vx.out_limit = UPF_ANGLE_LIMIT;
    pid_upf_vx.integral = 0.0f;
    pid_upf_vx.last_error = 0.0f;
    pid_upf_vx.d_lpf_alpha = 0.25f;

    /* ---- Y 轴速度阻尼 PI ---- */
    pid_upf_vy.kp = FLOW_VY_KP;
    pid_upf_vy.ki = FLOW_VY_KI;
    pid_upf_vy.kd = FLOW_VY_KD;
    pid_upf_vy.i_limit = 8.0f;     // 10→20，给积分足够空间抗恒力
    pid_upf_vy.out_limit = UPF_ANGLE_LIMIT;
    pid_upf_vy.integral = 0.0f;
    pid_upf_vy.last_error = 0.0f;
    pid_upf_vy.d_lpf_alpha = 0.25f;

#if FLOW_USE_LADRC
    ladrc_x.z1 = 0.0f; ladrc_x.z2 = 0.0f; ladrc_x.u_sat = 0.0f;
    ladrc_y.z1 = 0.0f; ladrc_y.z2 = 0.0f; ladrc_y.u_sat = 0.0f;
#endif
}


/* ==================== Up_Flow_302_Update ====================
 *
 * 这里和 PMW3901 实现最大的不同：
 *   PMW3901 是同步 SPI 读，每次调用一定有"新数据"。
 *   LC-302 是 UART 异步上报，可能 20ms 内来了 1 帧、0 帧或 2 帧。
 *   所以根据 upflow302_finsh_flag 决定是不是真的算了一帧新速度。
 *
 * @param height_cm  DL1B 测得的当前高度（cm）
 */
void Up_Flow_302_Update(float height_cm)
{
    /* ---- V1(旧)：陀螺仪 LPF 持续更新，V2 模式跳过 ---- */
#if UPF_USE_ROTATION_COMP && !UPF_USE_ANGLE_DOMAIN_COMP
    {
        const float gyro_x_rad_s_dbg = (float)last_gyro.imu660ra_gyro_x * UPF_GYRO_LSB_TO_RAD_S;
        const float gyro_y_rad_s_dbg = (float)last_gyro.imu660ra_gyro_y * UPF_GYRO_LSB_TO_RAD_S;
        const float gyro_z_rad_s_dbg = (float)last_gyro.imu660ra_gyro_z * UPF_GYRO_LSB_TO_RAD_S;

        upf_gyro_lpf_x = upf_lpf1(upf_gyro_lpf_x, gyro_x_rad_s_dbg, UPF_GYRO_LPF_HZ, UPF_UPDATE_DT_S);
        upf_gyro_lpf_y = upf_lpf1(upf_gyro_lpf_y, gyro_y_rad_s_dbg, UPF_GYRO_LPF_HZ, UPF_UPDATE_DT_S);
        upf_gyro_lpf_z = upf_lpf1(upf_gyro_lpf_z, gyro_z_rad_s_dbg, UPF_GYRO_LPF_HZ, UPF_UPDATE_DT_S);
        upf_dbg_gyro_lpf_x = upf_gyro_lpf_x;
        upf_dbg_gyro_lpf_y = upf_gyro_lpf_y;
        upf_dbg_gyro_lpf_z = upf_gyro_lpf_z;
    }
#endif

    /* ---- V2(新)：快照已在 2ms PIT 中断里由 Up_Flow_302_Gyro_Accum_2ms 完成 ---- */
    /* ======== ① 高度门控 ========
     * 高度无效时不直接清零速度，按比例衰减避免控制量突跳 */
    if (height_cm < UPF_MIN_HEIGHT_CM || height_cm > UPF_MAX_HEIGHT_CM)
    {
        upf_data_valid = 0;
        upf_data_fresh = 0;

        if (upf_invalid_streak < 250) upf_invalid_streak++;
        float decay = (upf_invalid_streak > 5) ? 0.90f : 0.97f;

        upf_vel_x *= decay;
        upf_vel_y *= decay;
        upf_raw_vel_x *= decay;
        upf_raw_vel_y *= decay;
        upf_lpf_vel_x *= decay;
        upf_lpf_vel_y *= decay;
        upf_dbg_raw_vx = upf_raw_vel_x;
        upf_dbg_raw_vy = upf_raw_vel_y;

        upf_pitch_corr = 0.0f;
        upf_roll_corr = 0.0f;

        return;
    }

    /* ======== ② 检测新帧 ========
     * upflow302_finsh_flag 由底层 callback 在 XOR 校验通过后置 1。
     * 我们读到 1 之后立即清零，让底层重新置位下一帧。
     * 注意：upflow302_receive 在底层是直接 memcpy 得到的，字段值在我们读之间可能被覆盖，
     *      但每帧 20~50ms 一次，主循环 20ms 一次，最多覆盖一次，可以接受。 */
    uint8 had_new_frame = 0;
    int16  flow_x_raw = 0;
    int16  flow_y_raw = 0;
    uint16 timespan_us = 0;
    uint8  valid_byte = 0;

    if (upflow302_finsh_flag)
    {
        upflow302_finsh_flag = 0;
        had_new_frame = 1;

        /* 把底层结构体的字段拷成本地变量，避免在做浮点运算时再被 ISR 改写
         *
         * 【硬件安装方向】（地面测试实测确认）：
         *   LC-302 模块 X 轴 → 飞机左右向（roll 平面）
         *   LC-302 模块 Y 轴 → 飞机前后向（pitch 平面）
         * 和应用层约定（X=前后 对应 pitch / Y=左右 对应 roll）相反，所以这里互换：
         *   应用层 X（前后向）取模块 Y
         *   应用层 Y（左右向）取模块 X
         * 如果以后旋转 LC-302 安装 90° 再装，把这里改回 .x/.y 对应即可。 */
        flow_x_raw  = upflow302_receive.upflow302_y;
        flow_y_raw  = upflow302_receive.upflow302_x;
        /* upflow302_us 字段在库里是 int16，但实际是无符号微秒数，强制转 uint16 */
        timespan_us = (uint16)upflow302_receive.upflow302_us;
        valid_byte  = upflow302_receive.upflow302_valid;

        upf_dbg_frame_cnt++;
        upf_dbg_last_frame_ms = sys_time_ms;

        /* 更新跨调用 valid 状态。无新帧的轮次不会改这个标志，
         * 所以"上一帧 invalid 之后没新帧"不会被误判成有效 */
        upf_last_frame_valid = (valid_byte == UPF_VALID_VAL) ? 1 : 0;
    }

    /* ======== ③ 帧超时 ========
     * LC-302 大约 20~50Hz。超过 200ms 没新帧认为通讯断或固件异常 */
    uint32 since_frame_ms = (sys_time_ms >= upf_dbg_last_frame_ms)
                          ? (sys_time_ms - upf_dbg_last_frame_ms)
                          : 0;
    if (upf_dbg_frame_cnt == 0 || since_frame_ms > UPF_FRAME_TIMEOUT_MS)
    {
        upf_data_valid = 0;
        Up_Flow_302_Update_Fresh_Flag();
        upf_vel_x *= 0.90f;
        upf_vel_y *= 0.90f;
        upf_raw_vel_x *= 0.90f;
        upf_raw_vel_y *= 0.90f;
        upf_lpf_vel_x *= 0.90f;
        upf_lpf_vel_y *= 0.90f;
        upf_dbg_raw_vx = upf_raw_vel_x;
        upf_dbg_raw_vy = upf_raw_vel_y;
        upf_pitch_corr = 0.0f;
        upf_roll_corr = 0.0f;
        return;
    }

    upf_invalid_streak = 0;

    /* ======== ③.5 新帧但 invalid → 衰减、清积分、退出 ========
     * Sophia 提的关键 bug 修复：旧实现里 invalid 帧只标 data_valid=0，
     * 但速度数据照样被 LPF 更新，下一帧没新数据时 data_valid 又被
     * 错误地置回 1，会让控制吃到不该吃的速度。
     * 这里直接 return，速度做指数衰减、PI 积分清零。 */
    if (had_new_frame && !upf_last_frame_valid)
    {
        upf_data_valid = 0;
        Up_Flow_302_Update_Fresh_Flag();
        upf_vel_x *= 0.90f;
        upf_vel_y *= 0.90f;
        upf_raw_vel_x *= 0.90f;
        upf_raw_vel_y *= 0.90f;
        upf_lpf_vel_x *= 0.90f;
        upf_lpf_vel_y *= 0.90f;
        upf_dbg_raw_vx = upf_raw_vel_x;
        upf_dbg_raw_vy = upf_raw_vel_y;
        upf_pitch_corr = 0.0f;
        upf_roll_corr = 0.0f;
        return;
    }

    /* ======== ④ 物理速度换算（仅在有新帧 + valid + timespan 合理时） ========
     *
     * 步骤：
     *   ④.1 角速度  omega = (pixel_integral / 10000) / (timespan_us × 1e-6) (rad/s)
     *   ④.2 旋转补偿（PDF "提升性能核心"）：减掉飞机自身旋转贡献
     *       omega_x_corr = omega_x + LIMIT(gyro_lpf_y_rad_s, ±lim)
     *       omega_y_corr = omega_y - LIMIT(gyro_lpf_x_rad_s, ±lim)
     *       目的：飞机只旋转不平移时 omega_corr ≈ 0
     *   ④.3 物理速度 v = omega_corr × height_cm                    (cm/s)
     *
     * 检验：1m 高、平移 50 cm/s，raw_x 应该在 50 附近。
     * 量级离谱先怀疑 timespan_us 字段；补偿后旋转飞机时速度不为零先怀疑符号。
     *
     * timespan 上下限门控：异常小的 timespan 会被 100/timespan 公式放大成
     * 天文数字，比单纯 fabsf(vel) > MAX 更可靠——异常从源头就拦住。
     */
    if (had_new_frame && upf_last_frame_valid &&
        (timespan_us < UPF_MIN_TIMESPAN_US ||
         timespan_us > UPF_MAX_TIMESPAN_US))
    {
        upf_last_frame_valid = 0;
        upf_data_valid = 0;
        Up_Flow_302_Update_Fresh_Flag();

        upf_vel_x *= 0.90f;
        upf_vel_y *= 0.90f;
        upf_raw_vel_x *= 0.90f;
        upf_raw_vel_y *= 0.90f;
        upf_lpf_vel_x *= 0.90f;
        upf_lpf_vel_y *= 0.90f;
        upf_dbg_raw_vx = upf_raw_vel_x;
        upf_dbg_raw_vy = upf_raw_vel_y;

        upf_pitch_corr = 0.0f;
        upf_roll_corr = 0.0f;

        return;
    }

    if (had_new_frame && upf_last_frame_valid &&
        timespan_us >= UPF_MIN_TIMESPAN_US &&
        timespan_us <= UPF_MAX_TIMESPAN_US)
    {
        const float dt_s = (float)timespan_us * 1e-6f;

#if UPF_USE_ANGLE_DOMAIN_COMP
        /* ======== V2(新)：角度域旋转补偿 ========
         * ① flow_angle = pixel / 10000          (光流总角位移, rad)
         * ② gyro_delta = gyro_angle(now) - gyro_angle(start)  (同窗口陀螺转角, rad)
         * ③ trans_angle = flow_angle + X_SIGN × gyro_delta    (纯平移角位移, rad)
         * ④ omega_corr = trans_angle / dt_s                   (补偿后角速度, rad/s)
         * ⑤ range = H / (cos(pitch)×cos(roll))                (沿光轴有效距离, cm)
         * ⑥ v = SCALE × SIGN × omega_corr × range             (物理速度, cm/s) */
        const float flow_angle_x = (float)flow_x_raw / UPF_PIXEL_INTEGRAL_SCALE;  /* rad */
        const float flow_angle_y = (float)flow_y_raw / UPF_PIXEL_INTEGRAL_SCALE;

        /* 查环形缓冲得到积分窗口起止时刻的陀螺累积角 */
        uint32_t now_ms = sys_time_ms;
        uint32_t period_ms = (uint32_t)timespan_us / 1000U;
        uint32_t start_ms = (period_ms >= now_ms) ? 0U : now_ms - period_ms;

        float gyro_pitch_start = gyro_buf_get_pitch_at(start_ms);
        float gyro_pitch_now   = gyro_buf_get_pitch_at(now_ms);
        float gyro_roll_start  = gyro_buf_get_roll_at(start_ms);
        float gyro_roll_now    = gyro_buf_get_roll_at(now_ms);

        /* 用最新的累积值覆盖 now 值（比缓冲插值更实时） */
        gyro_pitch_now = upf_gyro_angle_x;
        gyro_roll_now  = upf_gyro_angle_y;

        /* 同一窗口内陀螺总转角 */
        float gyro_delta_x = gyro_pitch_now - gyro_pitch_start;  /* pitch 角位移 */
        float gyro_delta_y = gyro_roll_now  - gyro_roll_start;   /* roll  角位移 */

        /* 角度域做差（符号与厂商代码一致） */
        float trans_angle_x = flow_angle_x + UPF_GYRO_COMP_X_SIGN * gyro_delta_x;
        float trans_angle_y = flow_angle_y + UPF_GYRO_COMP_Y_SIGN * gyro_delta_y;

        /* 转回角速度 */
        const float omega_x_corr = trans_angle_x / dt_s;
        const float omega_y_corr = trans_angle_y / dt_s;

        upf_dbg_gyro_lpf_x = gyro_delta_y / dt_s;  /* 借调试变量显示陀螺角速度 */
        upf_dbg_gyro_lpf_y = gyro_delta_x / dt_s;

#elif UPF_USE_ROTATION_COMP
        /* ======== V1(旧)：角速度域 + LPF 匹配补偿 ======== */
        const float omega_x = ((float)flow_x_raw / UPF_PIXEL_INTEGRAL_SCALE) / dt_s;
        const float omega_y = ((float)flow_y_raw / UPF_PIXEL_INTEGRAL_SCALE) / dt_s;
        const float lim = UPF_GYRO_COMP_LIMIT_RAD_S;
        const float comp_y = (upf_gyro_lpf_y >  lim) ?  lim
                           : (upf_gyro_lpf_y < -lim) ? -lim
                           : upf_gyro_lpf_y;
        const float comp_x = (upf_gyro_lpf_x >  lim) ?  lim
                           : (upf_gyro_lpf_x < -lim) ? -lim
                           : upf_gyro_lpf_x;
        const float comp_z = (upf_gyro_lpf_z >  lim) ?  lim
                           : (upf_gyro_lpf_z < -lim) ? -lim
                           : upf_gyro_lpf_z;
        const float omega_x_corr = omega_x + UPF_GYRO_COMP_X_SIGN * comp_y + UPF_GYRO_COMP_Z_SIGN * comp_z;
        const float omega_y_corr = omega_y + UPF_GYRO_COMP_Y_SIGN * comp_x + UPF_GYRO_COMP_Z_SIGN * comp_z;
#else
        /* 无旋转补偿 */
        const float omega_x = ((float)flow_x_raw / UPF_PIXEL_INTEGRAL_SCALE) / dt_s;
        const float omega_y = ((float)flow_y_raw / UPF_PIXEL_INTEGRAL_SCALE) / dt_s;
        const float omega_x_corr = omega_x;
        const float omega_y_corr = omega_y;
#endif /* rotation comp method selection */

        upf_dbg_omega_x = omega_x_corr;
        upf_dbg_omega_y = omega_y_corr;

        /* ④.3 角速度 × 高度 → 物理速度
         * 仰角补偿：倾斜时沿光轴有效距离 = H / cos(pitch) / cos(roll) */
        float eff_range = height_cm;
#if UPF_USE_TILT_COMP
        {
            float cp = cosf(fabsf(current_euler.pitch * DEG2RAD));
            float cr = cosf(fabsf(current_euler.roll  * DEG2RAD));
            if (cp > 0.1f && cr > 0.1f) {
                eff_range = height_cm / (cp * cr);
            }
        }
#endif
        upf_raw_vel_x = FLOW_VEL_SCALE * UPF_VEL_X_SIGN * omega_x_corr * eff_range;
        upf_raw_vel_y = FLOW_VEL_SCALE * UPF_VEL_Y_SIGN * omega_y_corr * eff_range;

        /* 跳变剔除 */
        if (fabsf(upf_raw_vel_x) > UPF_MAX_VEL_CMS) upf_raw_vel_x = upf_lpf_vel_x;
        if (fabsf(upf_raw_vel_y) > UPF_MAX_VEL_CMS) upf_raw_vel_y = upf_lpf_vel_y;

        upf_dbg_raw_vx = upf_raw_vel_x;
        upf_dbg_raw_vy = upf_raw_vel_y;
        upf_last_valid_ms = sys_time_ms;
    }

    /* ======== ⑤ 一阶低通滤波（每 20ms 调用都更新） ========
     * 没新帧时 raw 仍是上一次的值，相当于 LPF 平滑趋向旧值 */
    upf_lpf_vel_x += UPF_VEL_LPF_ALPHA * (upf_raw_vel_x - upf_lpf_vel_x);
    upf_lpf_vel_y += UPF_VEL_LPF_ALPHA * (upf_raw_vel_y - upf_lpf_vel_y);
    upf_vel_x = upf_lpf_vel_x;
    upf_vel_y = upf_lpf_vel_y;

    /* ======== ⑥ valid 状态输出 ========
     * 关键：用 upf_last_frame_valid 而不是 had_new_frame 的 else 分支
     * 旧实现 bug：没新帧时直接进 else 把 upf_data_valid 强制置 1，
     * 即使上一帧是 invalid 也会被"复活"。这里直接沿用上一帧的 valid
     * 状态，确保 invalid 帧之后没新帧时不会被误判为有效。 */
    upf_data_valid = upf_last_frame_valid;
    Up_Flow_302_Update_Fresh_Flag();
}


/* ==================== 速度指令 mock（阶段1调试用）====================
 * 不接真视觉，按时间生成一段测试速度指令，验证"速度环能否跟踪非零设定值"。
 * 循环 4 段（各 CAM_VEL_MOCK_PHASE_S 秒）：悬停 → 前进 → 悬停 → 后退。
 * 只测 pitch(前后)轴；想测 roll(左右)，把下面写 upf_target_vx 的地方换成 upf_target_vy。
 * @param dt 调用周期(秒)
 */
void Cam_Vel_Mock_Update(float dt)
{
    /* 由 main 按 FOLLOW_STEP 决定调不调：只有第2步(速度跟踪)会调用本函数。*/
    cam_mock_t += dt;
    /* 用累计时间算当前处于第几段(0..3循环) */
    int phase = ((int)(cam_mock_t / CAM_VEL_MOCK_PHASE_S)) % 4;
    switch (phase)
    {
        case 0: upf_target_vx = 0.0f;                 break;  /* 悬停 */
        case 1: upf_target_vx = +CAM_VEL_MOCK_SPEED;  break;  /* 前进 */
        case 2: upf_target_vx = 0.0f;                 break;  /* 悬停 */
        case 3: upf_target_vx = -CAM_VEL_MOCK_SPEED;  break;  /* 后退 */
        default: upf_target_vx = 0.0f;                break;
    }
    upf_target_vy = 0.0f;          /* 只测前后轴，左右目标保持 0 */
}

/* 复位 mock：计时归零 + 目标速度归零（纯悬停），位置 mock 状态也一并清零。
 * 跟车未就绪 / 离开定高时每周期调用，保证就绪那一刻从干净状态开始。*/
void Cam_Vel_Mock_Reset(void)
{
    cam_mock_t = 0.0f;
    upf_target_vx = 0.0f;
    upf_target_vy = 0.0f;
    /* 第3步 位置 mock 状态 */
    cam_drone_x = 0.0f;
    cam_drone_y = 0.0f;
#if VCAR_MODE == 1
    cam_vcar_t  = 0.0f;
#endif
    cam_rel_x = 0.0f;
    cam_rel_y = 0.0f;
    cam_valid = 0;
}

/* ==================== 第3步：虚拟车位置跟随（备用）====================
 * (A) 数据源：用虚拟车 + 积分光流位置，模拟真相机会输出的"车相对飞机坐标"。
 *     真相机就绪后，删掉本函数，改成从共享内存读 cam_rel_x/y（并用 IMU 姿态去旋转）。
 * @param dt 调用周期(秒)
 */
void Cam_Pos_Mock_Update(float dt)
{
    /* 1. 积分光流速度 → 飞机世界位移（没有相机时的航位推算；会慢慢漂，属正常，真相机更准）
     *    加固：仅在光流速度可信时积分，无效帧冻结；位移限幅，避免丢帧/尖刺把位置积飞。*/
    if (upf_data_valid)
    {
        cam_drone_x += upf_vel_x * dt;
        cam_drone_y += upf_vel_y * dt;
        cam_drone_x = f_limit(cam_drone_x, -POSHOLD_MAX_CM, POSHOLD_MAX_CM);
        cam_drone_y = f_limit(cam_drone_y, -POSHOLD_MAX_CM, POSHOLD_MAX_CM);
    }

    /* 2. 虚拟车世界位置：静止(阶跃) 或 正弦往返 */
    float vcar_x = 0.0f;
    float vcar_y = 0.0f;
#if VCAR_MODE == 1
    cam_vcar_t += dt;
    vcar_x = VCAR_SINE_AMP * sinf(2.0f * 3.14159265f * cam_vcar_t / VCAR_SINE_PERIOD);
#else
    vcar_x = VCAR_STEP_X;     /* 车固定在世界系 X=VCAR_STEP_X，飞机阶跃飞过去并停住 */
#endif

    /* 3. 相对坐标 = 车 - 飞机（这就是真相机会测到的"车相对飞机"）*/
    cam_rel_x = vcar_x - cam_drone_x;
    cam_rel_y = vcar_y - cam_drone_y;
    cam_valid = 1;
}

/* (B) 跟随外环（P + 前馈）：相对坐标(误差) → 期望速度。
 *
 *  cx/cy 已经过仰角补偿，位置测量不包含姿态耦合。
 *  绳子只影响姿态和速度（物理层面），不影响位置测量。
 *  所以位置环不需要 LPF/陷波——它看到的误差就是真实的相对位置。
 *
 *  P(0.90)：直接响应位置误差，回归目标。
 *  FF(0.55)：误差变化率→速度偏置，车突然转向时预判响应。
 *  速度环FLOW_VX_KP=0.50：提供实时阻尼，频率无关。
 */
void Cam_Follow_Outer_Update(float dt)
{
    static float last_ex = 0.0f, last_ey = 0.0f;
    static float ff_x = 0.0f, ff_y = 0.0f;

    if (!cam_valid)
    {
        upf_target_vx = 0.0f;
        upf_target_vy = 0.0f;
        last_ex = 0.0f; last_ey = 0.0f;
        ff_x = 0.0f; ff_y = 0.0f;
        return;
    }

    /* 轴映射：摄像头X→光流Y负、摄像头Y→光流X正 */
    float err_x =  cam_rel_y;
    float err_y = -cam_rel_x;
    if (fabsf(err_x) < CAM_DEADBAND_CM) err_x = 0.0f;
    if (fabsf(err_y) < CAM_DEADBAND_CM) err_y = 0.0f;

    /* 前馈：err变化率 → 速度偏置 */
    float raw_ff_x = (err_x - last_ex) / dt;
    float raw_ff_y = (err_y - last_ey) / dt;
    last_ex = err_x; last_ey = err_y;
    ff_x += CAM_FF_LPF_ALPHA * (raw_ff_x * CAM_FF_GAIN - ff_x);
    ff_y += CAM_FF_LPF_ALPHA * (raw_ff_y * CAM_FF_GAIN - ff_y);

    float vx_des = CAM_POS_KP * err_x + ff_x;
    float vy_des = CAM_POS_KP * err_y + ff_y;
    upf_target_vx = f_limit(vx_des, -V_FOLLOW_MAX, V_FOLLOW_MAX);
    upf_target_vy = f_limit(vy_des, -V_FOLLOW_MAX, V_FOLLOW_MAX);
}

/* ==================== C2：起飞/悬停位置保持外环 ====================
 * poshold_x/y = 相对"起飞点(原点)"的位移估计(cm)，由滤波光流速度积分得到。
 * 设计要点：
 *   ① 只有 upf_data_valid 时才积分；无效帧冻结(不动)，避免把噪声/丢帧积成假位移。
 *   ② 漏积分：每步乘 POSHOLD_LEAK(<1)，把蓝布上光流的随机游走累积慢慢"忘掉"，
 *      使位移估计有界、不会越积越大把飞机带飞。
 *   ③ 位移限幅 ±POSHOLD_MAX_CM，再外环 P 转成期望速度并限幅，喂给现有速度环。
 * 误差 = 0 - pos = -pos（期望停在原点），所以期望速度 = -KP*pos。*/
static float poshold_x = 0.0f;   /* 相对起飞点位移 X(前后, cm) */
static float poshold_y = 0.0f;   /* 相对起飞点位移 Y(左右, cm) */

void Up_Flow_302_PosHold_Update(float dt)
{
    /* ① 仅在速度可信时积分位移；② 漏积分防随机游走；③ 限幅 */
    if (upf_data_valid)
    {
        poshold_x += upf_vel_x * dt;
        poshold_y += upf_vel_y * dt;
        poshold_x *= POSHOLD_LEAK;
        poshold_y *= POSHOLD_LEAK;
        poshold_x = f_limit(poshold_x, -POSHOLD_MAX_CM, POSHOLD_MAX_CM);
        poshold_y = f_limit(poshold_y, -POSHOLD_MAX_CM, POSHOLD_MAX_CM);
    }
    /* 无效帧：冻结积分(保持 poshold 不变)。此时 Speed_Damp 也会因 fresh 失效而输出 0。 */

    /* 外环 P：位置误差(-pos) → 期望速度，再限幅 */
    float vx_des = -POSHOLD_KP * poshold_x;
    float vy_des = -POSHOLD_KP * poshold_y;
    upf_target_vx = f_limit(vx_des, -POSHOLD_V_MAX, POSHOLD_V_MAX);
    upf_target_vy = f_limit(vy_des, -POSHOLD_V_MAX, POSHOLD_V_MAX);

    /* 复用调试打印：rx/ry 现在表示"相对起飞点位移估计(cm)" */
    cam_rel_x = poshold_x;
    cam_rel_y = poshold_y;
}

void Up_Flow_302_PosHold_Reset(void)
{
    poshold_x = 0.0f;
    poshold_y = 0.0f;
    upf_target_vx = 0.0f;
    upf_target_vy = 0.0f;
    cam_rel_x = 0.0f;
    cam_rel_y = 0.0f;
}

/* ==================== LADRC 一阶自抗扰控制器 ====================
 *
 * 一阶系统模型：v_dot = b0 * u + f
 *   v = 速度(cm/s), u = 角度指令(°), f = 总扰动(绳拉力/风等, cm/s²)
 *
 * ESO(扩张状态观测器) 离散化(前向欧拉 @ 100Hz)：
 *   e = z1 - y
 *   z1 += dt * (z2 + b0*u_sat - 2*wo*e)
 *   z2 += dt * (-wo²*e)
 *   z1→估计速度, z2→估计总扰动
 *
 * 控制律：u = (wc*(target - z1) - z2) / b0
 *   第一部分：PD反馈(追目标)，第二部分：前馈补偿(抵消扰动)
 *
 * 相比纯P：
 *   纯P：out = KP*(target-v) → 需要等v因扰动变化后才出力 → 滞后
 *   LADRC：z2在线估计扰动 → u = (-z2)/b0 直接抵消 → 不等待
 */
#if FLOW_USE_LADRC
typedef struct {
    float z1;       /* 估计速度 (cm/s) */
    float z2;       /* 估计总扰动 (cm/s²) */
    float u_sat;    /* 上次饱和输出 (°) */
} ladrc_axis_t;

static ladrc_axis_t ladrc_x = {0, 0, 0};
static ladrc_axis_t ladrc_y = {0, 0, 0};

/* LADRC 单轴更新 (dt 单位: 秒, limit 单位: °) */
static float ladrc_update(ladrc_axis_t *s, float target, float y, float dt, float limit)
{
    /* ESO 观测误差 */
    float e = s->z1 - y;
    /* ESO 更新（用上次饱和后的输出，避免 windup） */
    s->z1 += dt * (s->z2 + LADRC_B0 * s->u_sat - 2.0f * LADRC_WO * e);
    s->z2 += dt * (-LADRC_WO * LADRC_WO * e);
    /* 控制律：误差反馈 + 扰动前馈补偿 */
    float u = (LADRC_WC * (target - s->z1) - s->z2) / LADRC_B0;
    /* 限幅并保存（下次 ESO 用 u_sat） */
    s->u_sat = f_limit(u, -limit, limit);
    return s->u_sat;
}
#endif /* FLOW_USE_LADRC */

/* ==================== Up_Flow_302_Speed_Damp ==================== */
void Up_Flow_302_Speed_Damp(float dt, float *pitch_corr, float *roll_corr)
{
    if (pitch_corr == NULL || roll_corr == NULL) return;
    *pitch_corr = 0.0f;
    *roll_corr  = 0.0f;

    if (!upf_data_fresh)
    {
    #if FLOW_USE_LADRC
        /* LADRC：z1跟随当前速度，恢复时无跳变；z2保留（扰动估计不突变） */
        float vel_x = UPF_USE_IMU_FUSION ? upf_fused_vx : upf_vel_x;
        float vel_y = UPF_USE_IMU_FUSION ? upf_fused_vy : upf_vel_y;
        ladrc_x.z1 = vel_x;
        ladrc_y.z1 = vel_y;
        ladrc_x.u_sat = 0.0f;
        ladrc_y.u_sat = 0.0f;
    #else
        upf_pitch_corr = 0.0f;
        upf_roll_corr = 0.0f;
    #endif
        return;
    }

    /* 速度环：IMU-光流互补融合 + 旋转补偿 */
#if UPF_USE_IMU_FUSION
    float vel_x = upf_fused_vx;
    float vel_y = upf_fused_vy;
#else
    float vel_x = upf_vel_x;
    float vel_y = upf_vel_y;
#endif

#if FLOW_USE_LADRC
    /* ========== LADRC 速度环（保守参数：wc=2, wo=12, b0=15） ========== */
    float out_pitch = ladrc_update(&ladrc_x, upf_target_vx, vel_x, dt, UPF_ANGLE_LIMIT);
    float out_roll  = ladrc_update(&ladrc_y, upf_target_vy, vel_y, dt, UPF_ANGLE_LIMIT);

    /* 调试变量：误差 ≈ (target - z1) 的缩放，用于观测跟踪效果 */
    upf_dbg_err_vx = upf_target_vx - ladrc_x.z1;
    upf_dbg_err_vy = upf_target_vy - ladrc_y.z1;
    upf_dbg_raw_out_pitch = out_pitch;
    upf_dbg_raw_out_roll  = out_roll;
#else
    /* ========== 原 PID 速度环（回退路径） ========== */
    float err_vx = upf_target_vx - vel_x;
    float err_vy = upf_target_vy - vel_y;
    if (fabsf(err_vx) < FLOW_VEL_DEADBAND_CM_S) err_vx = 0.0f;
    if (fabsf(err_vy) < FLOW_VEL_DEADBAND_CM_S) err_vy = 0.0f;

    uint8_t near_target = (fabsf(current_height_cm) >= UPF_I_FREEZE_UNTIL_CM) ? 1U : 0U;
    uint8_t integral_frozen = !near_target || (fabsf(current_speed_z) > UPF_CLIMB_VZ_THRESHOLD) ? 1U : 0U;

    if (!integral_frozen && fabsf(err_vx) < UPF_INT_ERR_GATE_CMS)
    {
        pid_upf_vx.integral *= 0.99f;
        pid_upf_vx.integral += err_vx * dt;
        pid_upf_vx.integral = f_limit(pid_upf_vx.integral, -pid_upf_vx.i_limit, pid_upf_vx.i_limit);
    }
    if (!integral_frozen && fabsf(err_vy) < UPF_INT_ERR_GATE_CMS)
    {
        pid_upf_vy.integral *= 0.99f;
        pid_upf_vy.integral += err_vy * dt;
        pid_upf_vy.integral = f_limit(pid_upf_vy.integral, -pid_upf_vy.i_limit, pid_upf_vy.i_limit);
    }

    upf_dbg_err_vx = err_vx;
    upf_dbg_err_vy = err_vy;

    float raw_dvx = (err_vx - pid_upf_vx.last_error) / dt;
    pid_upf_vx.last_error = err_vx;
    pid_upf_vx.derivative = pid_upf_vx.last_derivative + pid_upf_vx.d_lpf_alpha * (raw_dvx - pid_upf_vx.last_derivative);
    pid_upf_vx.last_derivative = pid_upf_vx.derivative;

    float raw_dvy = (err_vy - pid_upf_vy.last_error) / dt;
    pid_upf_vy.last_error = err_vy;
    pid_upf_vy.derivative = pid_upf_vy.last_derivative + pid_upf_vy.d_lpf_alpha * (raw_dvy - pid_upf_vy.last_derivative);
    pid_upf_vy.last_derivative = pid_upf_vy.derivative;

    float out_pitch = pid_upf_vx.kp * err_vx + pid_upf_vx.ki * pid_upf_vx.integral + pid_upf_vx.kd * pid_upf_vx.derivative;
    float out_roll  = pid_upf_vy.kp * err_vy + pid_upf_vy.ki * pid_upf_vy.integral + pid_upf_vy.kd * pid_upf_vy.derivative;

    out_pitch = f_limit(out_pitch, -UPF_ANGLE_LIMIT, UPF_ANGLE_LIMIT);
    out_roll  = f_limit(out_roll,  -UPF_ANGLE_LIMIT, UPF_ANGLE_LIMIT);

    upf_dbg_raw_out_pitch = out_pitch;
    upf_dbg_raw_out_roll  = out_roll;
#endif /* FLOW_USE_LADRC */

    *pitch_corr = UPF_CTRL_PITCH_SIGN * out_pitch;
    *roll_corr  = UPF_CTRL_ROLL_SIGN  * out_roll;
    upf_pitch_corr = *pitch_corr;
    upf_roll_corr  = *roll_corr;
}


/* ==================== Up_Flow_302_Reset ====================
 *
 * 落地/状态机回 IDLE / 急停时调用。
 * 不要只清 upf_vel_x/y —— LPF 历史也要清，否则下次起飞会带入旧速度。
 */
void Up_Flow_302_Reset(void)
{
    upf_vel_x = 0.0f;
    upf_vel_y = 0.0f;
    upf_raw_vel_x = 0.0f;
    upf_raw_vel_y = 0.0f;
    upf_lpf_vel_x = 0.0f;
    upf_lpf_vel_y = 0.0f;

    upf_dbg_raw_vx = 0.0f;
    upf_dbg_raw_vy = 0.0f;
#if UPF_USE_IMU_FUSION
    upf_imu_vx = 0.0f;
    upf_imu_vy = 0.0f;
    upf_fused_vx = 0.0f;
    upf_fused_vy = 0.0f;
#endif
    upf_dbg_omega_x = 0.0f;
    upf_dbg_omega_y = 0.0f;
    upf_dbg_gyro_lpf_x = 0.0f;
    upf_dbg_gyro_lpf_y = 0.0f;
    upf_dbg_gyro_lpf_z = 0.0f;
    upf_dbg_frame_cnt = 0;
    upf_dbg_last_frame_ms = sys_time_ms;

    upf_pitch_corr = 0.0f;
    upf_roll_corr = 0.0f;

    /* 速度环目标 + mock 计时清零：落地/急停后从悬停重新开始 */
    upf_target_vx = 0.0f;
    upf_target_vy = 0.0f;
    cam_mock_t = 0.0f;

    upf_data_valid = 0;
    upf_data_fresh = 0;
    upf_last_valid_ms = sys_time_ms - UPF_VALID_HOLD_MS;
    upf_last_frame_valid = 0;
    upf_invalid_streak = 0;

#if UPF_USE_ROTATION_COMP && !UPF_USE_ANGLE_DOMAIN_COMP
    upf_gyro_lpf_x = 0.0f;
    upf_gyro_lpf_y = 0.0f;
    upf_gyro_lpf_z = 0.0f;
#endif
#if UPF_USE_ANGLE_DOMAIN_COMP
    upf_gyro_angle_x = 0.0f;
    upf_gyro_angle_y = 0.0f;
    upf_gyro_buf_wr = 0;
    upf_gyro_buf_cnt = 0;
    for (uint8_t i = 0; i < UPF_GYRO_BUF_SIZE; i++) {
        upf_gyro_buf[i].time_ms = sys_time_ms;
        upf_gyro_buf[i].pitch_angle = 0.0f;
        upf_gyro_buf[i].roll_angle  = 0.0f;
    }
#endif

    pid_upf_vx.integral = 0.0f;
    pid_upf_vx.last_error = 0.0f;
    pid_upf_vx.d_lpf_alpha = 0.25f;
    pid_upf_vy.integral = 0.0f;
    pid_upf_vy.last_error = 0.0f;
    pid_upf_vy.d_lpf_alpha = 0.25f;

#if FLOW_USE_LADRC
    ladrc_x.z1 = 0.0f; ladrc_x.z2 = 0.0f; ladrc_x.u_sat = 0.0f;
    ladrc_y.z1 = 0.0f; ladrc_y.z2 = 0.0f; ladrc_y.u_sat = 0.0f;
#endif

    /* C2：位置保持位移估计也清零，落地/急停后下次以新点为原点 */
    poshold_x = 0.0f;
    poshold_y = 0.0f;
}

/* 相机接管期间钉原点，丢目标回退时不朝旧原点猛冲 */
void Up_Flow_302_PosHold_Prime(void)
{
    poshold_x = 0.0f;
    poshold_y = 0.0f;
}

#endif /* enable switch */
