#include "Optical_flow.h"
#if 0   // PMW3901 optical-flow implementation disabled: switching to UP FLOW 302.
#include "zf_device_pmw3901.h"
#include "math.h"

/*
 * ==================== 本文件职责 ====================
 * 光流速度阻尼模块的实现。将 PMW3901 传感器的原始像素增量，
 * 经过 换算 → 去噪 → 滤波 → PI控制 后，输出姿态修正角。
 *
 * ==================== 调用时序（在 main 中） ====================
 *
 *   每 10ms 主循环
 *     ├── Altitude_Control_Task()     // 定高任务（更新 current_height_cm）
 *     ├── of_cnt++
 *     └── 每 20ms（of_cnt >= 2）
 *           ├── Optical_Flow_Update(current_height_cm)   // ① 读传感器、换算速度
 *           └── Optical_Flow_Speed_Damp(0.02, &pitch, &roll)  // ② PI 计算修正角
 *
 * 注意：Update 必须在 Speed_Damp 之前调用，因为 Damp 依赖 Update 产出的 of_vel_x/y
 *
 * ==================== 变量命名规则 ====================
 *   of_      = optical flow（光流）前缀
 *   raw_     = 原始换算值（未滤波）
 *   lpf_     = low-pass filtered（低通滤波后）
 *   vel_     = velocity（速度）
 *   _i       = integral（积分/累加）
 *   corr     = correction（修正量）
 */

/* ==================== 全局变量定义 ==================== */

/* 对外状态量 —— 其他模块可直接读取用于调试打印 */
float of_vel_x = 0.0f;        // X轴滤波后速度（cm/s），约定为前后方向，对应 pitch 修正
float of_vel_y = 0.0f;        // Y轴滤波后速度（cm/s），约定为左右方向，对应 roll 修正
uint8 of_data_valid = 0;      // 数据有效标志：1=速度可信，0=高度超限或传感器异常

int16 of_dbg_dx = 0;          // PMW3901 原始 delta_x，供方向验证打印
int16 of_dbg_dy = 0;          // PMW3901 原始 delta_y，供方向验证打印
float of_dbg_raw_vx = 0.0f;   // 换算后的原始前后速度（跳变剔除前）
float of_dbg_raw_vy = 0.0f;   // 换算后的原始左右速度（跳变剔除前）
uint16 of_zero_streak = 0;     // 连续 dx/dy 都为 0 的有效光流帧数

/* PI 控制器实例 —— 对外暴露，方便在串口调试时在线改 kp/ki */
PID_t pid_of_vx;              // X轴速度阻尼 PI（输出 → pitch 修正角）
PID_t pid_of_vy;              // Y轴速度阻尼 PI（输出 → roll 修正角）

/* 内部状态量 —— static 保护，只能在本文件内访问 */
static float of_raw_vel_x = 0.0f;     // X轴原始速度（跳变剔除前）
static float of_raw_vel_y = 0.0f;     // Y轴原始速度（跳变剔除前）
static float of_lpf_vel_x = 0.0f;     // X轴低通滤波后的速度
static float of_lpf_vel_y = 0.0f;     // Y轴低通滤波后的速度
static uint8 of_invalid_streak = 0;    // 连续高度无效帧计数（用于衰减策略）


/* ==================== Optical_Flow_Init ====================
 *
 * 功能：初始化光流模块的全部状态
 * 调用时机：上电后只调一次，在 pmw3901_init() 之后
 *
 * 当前参数说明：
 *   kp = 0.035 → 速度误差 1 cm/s 时输出 0.035 度修正角
 *                例如飘了 10 cm/s，修正角 = 0.035 × 10 = 0.35 度
 *   ki = 0.0   → 关闭积分（验证阶段，避免静止偏置被积分放大）
 *   kd = 0.0   → 不用微分（速度信号本身已有低通滤波，不需要 D 项）
 *   out_limit  → 修正角最大 ±2 度（验证阶段保守值）
 *
 * 开积分的时机：
 *   当 kp 调好后，如果发现悬停时有 1-2 cm/s 的稳态误差无法消除，
 *   可以把 ki 从小值（0.005）开始慢慢加，i_limit 设到 3~5 度。
 */
void Optical_Flow_Init(void)
{
    /* ---- X轴 PI（对应俯仰方向的速度阻尼） ---- */
    pid_of_vx.kp = 0.020f;
    pid_of_vx.ki = 0.0f;             // 验证阶段关闭积分
    pid_of_vx.kd = 0.0f;             // 不用微分
    pid_of_vx.i_limit = 0.0f;        // 积分限幅（ki=0 时无意义，开积分后设为 3~5）
    pid_of_vx.out_limit = OF_ANGLE_LIMIT;  // 输出限幅 ±1 度
    pid_of_vx.integral = 0.0f;

    /* ---- Y轴 PI（对应横滚方向的速度阻尼） ---- */
    pid_of_vy.kp = 0.023f;
    pid_of_vy.ki = 0.0f;
    pid_of_vy.kd = 0.0f;
    pid_of_vy.i_limit = 0.0f;
    pid_of_vy.out_limit = OF_ANGLE_LIMIT;
    pid_of_vy.integral = 0.0f;

    /* ---- 速度状态全部归零 ---- */
    of_vel_x = 0.0f;
    of_vel_y = 0.0f;
    of_raw_vel_x = 0.0f;
    of_raw_vel_y = 0.0f;
    of_lpf_vel_x = 0.0f;
    of_lpf_vel_y = 0.0f;
    of_data_valid = 0;
    of_dbg_dx = 0;
    of_dbg_dy = 0;
    of_dbg_raw_vx = 0.0f;
    of_dbg_raw_vy = 0.0f;
    of_zero_streak = 0;
    of_invalid_streak = 0;
}


/* ==================== Optical_Flow_Update ====================
 *
 * 功能：读取一帧光流数据 → 换算为物理速度 → 滤波 → 更新全局速度
 * 调用时机：每 20ms 由主循环调用一次
 * 前置条件：height_cm 由定高模块提供（current_height_cm）
 *
 * 处理流程：
 *   ① 高度门控 —— 高度无效时标记数据无效，对速度做衰减而非清零
 *   ② 读取原始数据 —— 调用 pmw3901_get_motion() 获取像素增量
 *   ③ 速度换算 —— 像素增量 × 系数 × 高度 / 时间 → 物理速度 (cm/s)
 *   ④ 跳变剔除 —— 超过 300cm/s 的帧视为噪声，用上一帧滤波值代替
 *   ⑤ 低通滤波 —— 一阶 IIR 滤波平滑速度
 *
 * @param height_cm  当前 TOF 测量高度（厘米），范围 10~220 cm
 */
void Optical_Flow_Update(float height_cm)
{
    /* ======== 第①步：高度门控 ========
     *
     * PMW3901 在以下情况不可靠：
     *   - 高度 < 10cm：镜头对焦模糊，近地面纹理太大
     *   - 高度 > 220cm：超出传感器有效识别范围
     *
     * 无效时不能直接把速度清零，否则高度一恢复，控制量会突然跳变。
     * 做法是对速度做逐帧衰减（乘以一个小于 1 的系数），
     * 这样速度会自然平滑地趋向零。
     */
    if (height_cm < OF_MIN_HEIGHT_CM || height_cm > OF_MAX_HEIGHT_CM)
    {
        of_data_valid = 0;

        /* 衰减策略：根据连续无效帧数选择衰减速率
         *   - 短暂遮挡（≤5帧 ≈ 100ms）：用 0.97 缓慢衰减，100ms 后保留约 74%
         *     这样 TOF 短暂丢失不会导致速度信息大幅丢失
         *   - 持续失效（>5帧）：用 0.90 加速衰减，约 1 秒后降到接近零
         *     避免旧的、已不可信的速度长时间残留在滤波器中 */
        if (of_invalid_streak < 250)
        {
            of_invalid_streak++;
        }
        float decay = (of_invalid_streak > 5) ? 0.90f : 0.97f;

        /* 所有速度状态同步衰减（外部可见 + 内部滤波器） */
        of_vel_x *= decay;
        of_vel_y *= decay;
        of_raw_vel_x *= decay;
        of_raw_vel_y *= decay;
        of_lpf_vel_x *= decay;
        of_lpf_vel_y *= decay;

        /* 高度无效时清掉本帧原始增量，避免日志误把上一帧 dx/dy 当成当前数据。 */
        of_dbg_dx = 0;
        of_dbg_dy = 0;
        of_dbg_raw_vx = of_raw_vel_x;
        of_dbg_raw_vy = of_raw_vel_y;
        of_zero_streak = 0;

        /* 清零 PI 积分：高度无效时停止积分，防止在无效数据上积累错误积分 */
        pid_of_vx.integral = 0.0f;
        pid_of_vy.integral = 0.0f;
        return;
    }

    /* 高度有效，重置无效帧计数，下次失效重新开始计数 */
    of_invalid_streak = 0;

    /* ======== 第②步：读取传感器原始数据 ========
     * pmw3901_get_motion() 通过 SPI 读取 6 字节，
     * 解析出 pmw3901_delta_x 和 pmw3901_delta_y（int16 类型，有符号像素增量）
     * 注意：该函数内部有累积积分（pmw3901_delta_x_i），本模块不使用 */
    pmw3901_get_motion();
    of_dbg_dx = pmw3901_delta_x;
    of_dbg_dy = pmw3901_delta_y;
    if (pmw3901_delta_x == 0 && pmw3901_delta_y == 0)
    {
        if (of_zero_streak < 1000) of_zero_streak++;
    }
    else
    {
        of_zero_streak = 0;
    }

    /* ======== 第③步：像素增量 → 物理速度换算 ========
     *
     * 坐标映射关系（根据手持平移实测修正）：
     *   PMW3901 的 raw_x → 飞机前后方向 → of_vel_x（俯仰轴）
     *   PMW3901 的 raw_y → 飞机左右方向 → of_vel_y（横滚轴）
     *
     * OF_VEL_X_SIGN / OF_VEL_Y_SIGN 用于最终校正符号方向。
     */
    {
        const float raw_x = (float)pmw3901_delta_x;   // int16 → float
        const float raw_y = (float)pmw3901_delta_y;

#if OF_USE_RAD_MODEL
        /* 方案B：角速度模型
         * scale = 0.0244 rad/count ÷ 0.02 s × height_cm
         * 单位：rad/count × count/s × cm = cm/s
         * 速度 = 像素增量 × scale */
        const float scale = OF_PMW3901_RAD_PER_COUNT / OF_UPDATE_DT_S * height_cm;
        of_raw_vel_x = OF_VEL_X_SIGN * (raw_x * scale);
        of_raw_vel_y = OF_VEL_Y_SIGN * (raw_y * scale);
#else
        /* 方案A：经验系数模型（当前默认）
         * scale = 0.0105 cm/count/cm × height_cm ÷ 0.02 s
         * 含义：在 height_cm 高度下，每 count 对应的物理速度 (cm/s)
         * 速度 = 像素增量 × scale */
        const float scale = OF_COUNT_TO_CMS_PER_CM * height_cm / OF_UPDATE_DT_S;
        of_raw_vel_x = OF_VEL_X_SIGN * (raw_x * scale);
        of_raw_vel_y = OF_VEL_Y_SIGN * (raw_y * scale);
#endif
        of_dbg_raw_vx = of_raw_vel_x;
        of_dbg_raw_vy = of_raw_vel_y;
    }

    /* ======== 第④步：跳变剔除 ========
     * PMW3901 在纹理缺失（纯白/纯黑地面）、光照突变、或高度突变时
     * 可能输出异常大的像素增量。如果不过滤，低通滤波器会被"污染"，
     * 导致后续多帧速度都偏大，产生姿态抖动。
     *
     * 处理方法：超过 OF_MAX_VEL_CMS (300 cm/s) 的帧，用上一帧的滤波值替代。
     * 这相当于"这帧数据不可信，沿用上一帧的结果"。
     */
    if (fabsf(of_raw_vel_x) > OF_MAX_VEL_CMS) of_raw_vel_x = of_lpf_vel_x;
    if (fabsf(of_raw_vel_y) > OF_MAX_VEL_CMS) of_raw_vel_y = of_lpf_vel_y;

    /* ======== 第⑤步：一阶低通滤波 ========
     * 公式：output = output + alpha × (input - output)
     * 等价于：output = alpha × input + (1-alpha) × last_output
     *
     * alpha = 0.35 时的频率特性：
     *   - 低频信号（真实运动，<2Hz）：几乎无衰减地通过
     *   - 高频信号（震动噪声，>10Hz）：衰减约 70%
     *   - 延迟约 1~2 帧（20~40ms），在可接受范围
     *
     * 如果飞行时发现速度响应太慢（飞机飘了才开始修正但修正滞后），
     * 可以把 alpha 从 0.35 增大到 0.5；如果噪声太大就减小到 0.2。
     */
    of_lpf_vel_x += OF_VEL_LPF_ALPHA * (of_raw_vel_x - of_lpf_vel_x);
    of_lpf_vel_y += OF_VEL_LPF_ALPHA * (of_raw_vel_y - of_lpf_vel_y);

    /* 将滤波结果赋给对外变量，供 main 中的串口打印和后续 PI 使用 */
    of_vel_x = of_lpf_vel_x;
    of_vel_y = of_lpf_vel_y;
    of_data_valid = 1;
}


/* ==================== Optical_Flow_Speed_Damp ====================
 *
 * 功能：基于速度误差计算姿态修正角（PI 控制器）
 * 调用时机：每 20ms，紧跟在 Optical_Flow_Update 之后
 *
 * 控制逻辑：
 *   目标速度 = 0（悬停不动）
 *   误差 = 目标速度 - 实际速度 = -of_vel_x
 *   修正角 = kp × 误差 + ki × 积分
 *
 * 举例：
 *   飞机以 10 cm/s 向前飘（of_vel_x = 10）
 *   误差 = -10
 *   修正角 = 0.035 × (-10) = -0.35 度
 *   → 通过 pitch 修正产生反向加速度，用来抵消前后漂移
 *
 * @param dt         调用周期（秒），通常传 OF_UPDATE_DT_S (0.02f)
 * @param pitch_corr [out] 俯仰修正角（度），输出给姿态外环
 * @param roll_corr  [out] 横滚修正角（度），输出给姿态外环
 */
void Optical_Flow_Speed_Damp(float dt, float *pitch_corr, float *roll_corr)
{
    /* 安全检查：指针为空时直接返回，不做任何操作 */
    if (pitch_corr == NULL || roll_corr == NULL) return;

    /* 先输出零值，后续如果有异常直接返回也不会输出脏数据 */
    *pitch_corr = 0.0f;
    *roll_corr = 0.0f;

    /* 光流数据无效时（高度超限 / 传感器异常），不输出任何修正量
     * 并清零积分，防止无效数据上的积分在恢复后造成姿态冲击 */
    if (!of_data_valid)
    {
        pid_of_vx.integral = 0.0f;
        pid_of_vy.integral = 0.0f;
        return;
    }

    /* ======== 速度误差计算 ========
     * 目标速度恒为 0（悬停），所以误差 = 0 - 当前速度 = -当前速度
     * err_vx > 0 表示飞机在向 X 负方向飘，需要向 X 正方向修正
     * err_vy > 0 表示飞机在向 Y 负方向飘，需要向 Y 正方向修正 */
    {
        const float err_vx = -of_vel_x;
        const float err_vy = -of_vel_y;

        /* ======== 积分项（当前关闭，ki=0） ========
         * 积分分离：只在误差小于 OF_INT_ERR_GATE_CMS (120 cm/s) 时才累积积分。
         * 目的：飞机快速移动时（误差大）不累积积分，只有接近悬停状态（误差小）
         *       时才用积分消除稳态误差，防止积分暴走（anti-windup）。
         *
         * i_limit 限制了积分最大值，相当于限制了积分项的最大贡献量。
         * 当前 ki=0，这段代码不会生效，但已经写好，开启积分时无需改动。 */
        if (fabsf(err_vx) < OF_INT_ERR_GATE_CMS)
        {
            pid_of_vx.integral += err_vx * dt;
            pid_of_vx.integral = f_limit(pid_of_vx.integral, -pid_of_vx.i_limit, pid_of_vx.i_limit);
        }
        if (fabsf(err_vy) < OF_INT_ERR_GATE_CMS)
        {
            pid_of_vy.integral += err_vy * dt;
            pid_of_vy.integral = f_limit(pid_of_vy.integral, -pid_of_vy.i_limit, pid_of_vy.i_limit);
        }

        /* ======== PI 输出计算 ========
         * out = kp × err + ki × integral
         * 然后限幅到 ±OF_ANGLE_LIMIT（1 度），防止光流模块输出过大修正量
         * 干扰正常姿态控制（起飞阶段尤为重要） */
        {
            float out_pitch = pid_of_vx.kp * err_vx + pid_of_vx.ki * pid_of_vx.integral;
            float out_roll = pid_of_vy.kp * err_vy + pid_of_vy.ki * pid_of_vy.integral;

            out_pitch = f_limit(out_pitch, -OF_ANGLE_LIMIT, OF_ANGLE_LIMIT);
            out_roll = f_limit(out_roll, -OF_ANGLE_LIMIT, OF_ANGLE_LIMIT);

            /* 最后乘以符号系数，补偿"速度→姿态角"的映射方向 */
            *pitch_corr = OF_CTRL_PITCH_SIGN * out_pitch;
            *roll_corr = OF_CTRL_ROLL_SIGN * out_roll;
        }
    }
}


/* ==================== Optical_Flow_Reset ====================
 *
 * 功能：清零全部光流状态（速度 + 滤波器 + 积分）
 * 调用时机：
 *   - 落地时（flight_state 切回 IDLE）
 *   - 急停/坠机保护触发时
 *   - 下次起飞前
 *
 * 为什么不能只清 of_vel_x/y？
 *   因为 LPF 滤波器（of_lpf_vel_x/y）里还保存着上一段的速度历史，
 *   如果不清，下次起飞时前几帧的滤波输出会带入旧的速度偏置，
 *   导致飞机刚起飞就向某个方向倾斜。
 */
void Optical_Flow_Reset(void)
{
    of_vel_x = 0.0f;
    of_vel_y = 0.0f;
    of_raw_vel_x = 0.0f;
    of_raw_vel_y = 0.0f;
    of_lpf_vel_x = 0.0f;
    of_lpf_vel_y = 0.0f;
    of_invalid_streak = 0;
    of_dbg_dx = 0;
    of_dbg_dy = 0;
    of_dbg_raw_vx = 0.0f;
    of_dbg_raw_vy = 0.0f;
    of_zero_streak = 0;
    of_data_valid = 0;

    pid_of_vx.integral = 0.0f;
    pid_of_vy.integral = 0.0f;
}
#endif
