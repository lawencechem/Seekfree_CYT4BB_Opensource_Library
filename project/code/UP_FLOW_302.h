#ifndef _UP_FLOW_302_H_
#define _UP_FLOW_302_H_

#include "zf_common_headfile.h"
#include "pid.h"

/*
 * 湖南优象 UP FLOW 302 (LC-302) 光流模块 —— 速度阻尼应用层
 *
 * ==================== 分层关系 ====================
 *   逐飞库 zf_device_upflow302：14B 帧的 UART 初始化、接收回调、XOR 校验、全局存储
 *     - upflow302_receive_init()       初始化 UART + 接收中断
 *     - upflow302_receive_callback()   UART ISR 里逐字节喂数据
 *     - upflow302_receive 结构体       最近一帧的解析结果
 *     - upflow302_finsh_flag           本帧 XOR 校验通过后置 1
 *     - upflow302_state_flag           通讯状态
 *     注意：库的 UART 号和引脚是在 zf_device_upflow302.h 里写死的宏，
 *           本工程把它们改成了 UART_6 / P03.1 / P03.0（手动 GBK 编辑）。
 *
 *   应用层（本文件 + UP_FLOW_302.c）：
 *     - 物理速度换算：像素积分 × 高度 / 积分时间
 *     - 一阶低通滤波
 *     - 速度阻尼 PI 控制器，输出 pitch/roll 修正角
 *     - UART 收帧/校验/存储完全委托给库
 *
 * ==================== 数据流 ====================
 *
 *   LC-302              UART6 ISR                     主循环 20ms
 *   ──────         ──────────────             ──────────────
 *   14B 帧 ──→ upflow302_receive_callback() (库的函数)
 *               ↓ XOR 校验通过
 *               upflow302_receive (库的全局结构体)
 *                       ↓
 *               Up_Flow_302_Update(height) ──→ upf_vel_x, upf_vel_y (cm/s)
 *               Up_Flow_302_Speed_Damp(dt)  ──→ pitch_corr, roll_corr (deg)
 *
 * ==================== 14 字节协议（参考 zf_device_upflow302） ====================
 *   [0]    head        0xFE
 *   [1]    device_id   0x0A
 *   [2-3]  x 像素积分   int16 LE
 *   [4-5]  y 像素积分   int16 LE
 *   [6-7]  积分时间     int16 LE (μs，正常 ~20000~50000)
 *   [8-9]  预留
 *   [10]   valid       245=有效，0=无效
 *   [11]   version
 *   [12]   XOR 校验 (字节 2..11 异或)
 *   [13]   帧尾
 *
 * ==================== UART 占用（在 vendor 库里改，不在本文件） ====================
 *   libraries/zf_device/zf_device_upflow302.h 第 51~54 行已手动改成：
 *     #define UP_FLOW_302_UART_INDEX   (UART_6)
 *     #define UP_FLOW_302_BAUDRATE     (19200)
 *     #define UP_FLOW_302_TX_PIN       (UART6_RX_P03_0)   // 模块 TX → MCU RX P03.0
 *     #define UP_FLOW_302_RX_PIN       (UART6_TX_P03_1)   // 模块 RX → MCU TX P03.1
 *   注意：必须用 GBK / ANSI 编码手动改（IAR / VSCode 设 GBK），
 *         不要用任何 UTF-8 工具去 patch，否则库里中文注释会被破坏。
 *   UART_6 注意：与 SPI3 资源冲突（库注释），但当前库版本未使用 SPI3，可用。
 *   cm7_0_isr.c 的 uart6_isr 里调 upflow302_receive_callback()（库的函数）。
 */

#if 1   /* 1=启用 UP FLOW 302；想暂时关掉换 0 */

/* ==================== 符号标定（与之前 PMW3901 思路一致） ====================
 * 地面测试实测约定：向前平移 → vx 正、向右平移 → vy 正
 * X/Y 互换在 .c 里 Update 函数处理；这里只管符号校正。
 * 实测：互换后向前移动时 vx 仍为负 → UPF_VEL_X_SIGN = -1.0 翻号
 *       向右移动时 vy 已经为正 → UPF_VEL_Y_SIGN = +1.0 不变 */
#define UPF_VEL_X_SIGN              (-1.0f)     /* 互换 X/Y 后 X 通道翻号，让"向前=正" */
#define UPF_VEL_Y_SIGN              (1.0f)      /* 互换 X/Y 后 Y 通道符号正确 */
#define UPF_CTRL_PITCH_SIGN         (-1.0f)    /* 速度→pitch 映射符号：由 pitch_flight_trim「向前漂增大来抑制」推得，
                                                * 前漂(vx>0) 需 pitch_corr>0，而 out_pitch<0，故取 -1。待飞行确认。 */
#define UPF_CTRL_ROLL_SIGN          (-1.0f)     /* C5：-1→+1→-1。实测rc始终≥0符号反了，改回-1 */

/* ==================== 有效性门限 ==================== */
#define UPF_MIN_HEIGHT_CM           (10.0f)
#define UPF_MAX_HEIGHT_CM           (300.0f)
#define UPF_MAX_VEL_CMS             (80.0f)     /* 单帧速度跳变上限：250→80。蓝布低纹理会蹦 130~170cm/s 的假速度，
                                                 * 灌进控制会导致修正饱和→正反馈炸机。本测试真实漂移<50，80 足够留余量。
                                                 * 超过即用上一帧滤波值替代(下方跳变剔除逻辑)。 */
#define UPF_VALID_VAL               (245)       /* upflow302_valid==245 视为有效 */
#define UPF_FRAME_TIMEOUT_MS        (200)       /* 超时无新帧 → 数据失效 */
#define UPF_VALID_HOLD_MS           (250U)      /* 150→250：蓝布纹理差频繁丢帧，拉长保持。500→250：太长会导致速度反馈过期，修正持续带着旧数据方向。折中。 */

/* timespan 上下限：正常 20000~50000us。
 * 太小（如刚初始化或解析错位）会被 100/timespan 公式放大成天文数字速度。
 * 太大则说明帧率过低（光照不足/低质量），结果不可信。
 * 上限取 60000，留在 uint16 范围内（library 字段是 int16，cast 后 uint16 最大 65535）。 */
#define UPF_MIN_TIMESPAN_US         (10000U)
#define UPF_MAX_TIMESPAN_US         (60000U)

/* ==================== 换算/滤波 ==================== */
#define UPF_UPDATE_DT_S             (0.01f)
#define FLOW_HOLD_ENABLE            (1)   // yaw 锁稳后重新打开光流：水平有控制，也能压住之前"漂出去带歪航向"的耦合
#define FLOW_POS_LOOP_ENABLE        (0)
#define FLOW_VEL_DAMP_ENABLE        (1)
#define FLOW_START_HEIGHT_CM        (10.0f)  // C1：30→20→10，光流在爬升段更早接管，起飞段就有水平控制
#define FLOW_FULL_HEIGHT_CM         (15.0f)  // C1：45→30→15，配合更早接管
#define FLOW_POS_KP                 (0.0f)
#define FLOW_VEL_KP                 (0.0f)
#define FLOW_VX_KI                  (0.00f)  /* 0.04→0：取消速度环I。I冻结到90cm且有泄漏，实际效果微弱，关掉简化。 */
#define FLOW_VY_KI                  (0.00f)  /* 0.06→0：同X */
#define FLOW_VX_KD                  (0.00f)  // 0.02→0：D项放大了旋转补偿残留的假速度跳变，关掉排除
#define FLOW_VY_KD                  (0.00f)  // 0.05→0.02→0：同上
#define FLOW_VEL_LPF_ALPHA          (0.15f)  // 0.35→0.15，增强滤波压制高度放大的光流噪声
#define FLOW_MAX_ANGLE_DEG          (12.0f)  // 18→12：手拿测试数据无效，回退原值
#define FLOW_VEL_DEADBAND_CM_S      (2.0f)   // 5→2。有旋转补偿了，死区不用那么大
#define FLOW_VEL_SCALE              (0.25f)  // 0.5→0.25：假速度随高度放大，降SCALE等效降假速度。
                                             // 同时提升Kp补回真实响应，让系统在高高度也能稳

#ifndef DEG2RAD
#define DEG2RAD  (3.14159265f / 180.0f)
#endif

/* ==================== IMU-光流 互补融合 ==================== */
#define UPF_USE_IMU_FUSION          (1)      // 0→1：IMU融合，光流丢帧时IMU补上，速度更平滑
#define UPF_FUSION_ALPHA            (0.15f)  // 互补滤波：IMU权重(悬停)
#define UPF_IMU_VEL_SCALE           (0.25f)  // IMU积分速度÷4，使其量级与光流匹配(实测imu≈4×flow)
#define UPF_CLIMB_ALPHA             (0.0f)   // 爬升/下降时融合系数：0=纯光流，防重力分离误差引起正反馈
#define UPF_ALPHA_SMOOTH            (0.05f)  // 爬升→悬停恢复时α平滑速率(~200ms时间常数)
#define UPF_ACC_LSB_TO_CMS2         (0.0598f) // ±2g量程：16384LSB/g → cm/s²/LSB
#define UPF_G_CMS2                  (980.665f) // 重力加速度 cm/s²

#define FLOW_VX_KP                  (0.50f)  // 0.70→0.50：降速环X轴P
#define FLOW_VY_KP                  (0.64f)  // 0.84→0.64：降速环Y轴P

/* ==================== C2：起飞/悬停位置保持外环（目标=起飞点原点）====================
 * 把滤波后的光流速度积分成"相对起飞点位移"，外环 P 把它拉回 0，输出期望速度喂给上面的速度环。
 * 这一步把"只减慢漂移的纯阻尼"升级成"会主动回原点的定点"。
 * 关键防护（见 UP_FLOW_302.c）：
 *   - 只有 upf_data_valid 时才积分，无效帧冻结，避免把噪声/丢帧积成假位移；
 *   - 漏积分(leak)：每步乘 POSHOLD_LEAK(<1)，限制蓝布噪声的随机游走累积；
 *   - 位置估计限幅 ±POSHOLD_MAX_CM，避免单帧尖刺把目标速度顶满。
 * 接真摄像头时：这套"自积分定点"换成相机直接给的相对位置即可，外环结构不变。*/
#define POSHOLD_KP                  (0.0f)    /* 1.0→0：光流只负责速度环，位置环交给摄像头 */

#define POSHOLD_V_MAX               (12.0f)   /* 回拉期望速度限幅 (cm/s)，与拴绳空间匹配 */
#define POSHOLD_LEAK                (0.997f)  /* 漏积分系数(每 ~10ms)，<1 防随机游走；越小忘得越快、定点越松 */
#define POSHOLD_MAX_CM              (60.0f)   /* 位移估计限幅 (cm) */

#define UPF_VEL_LPF_ALPHA           (FLOW_VEL_LPF_ALPHA)
#define UPF_PIXEL_INTEGRAL_SCALE    (10000.0f)  /* 协议固定放大系数 */

/* ==================== 角度域旋转补偿（V2：精确时间对齐） ====================
 *
 * 原理：光流传感器在积分窗口(20~50ms)内累积像素位移，陀螺仪应该在
 * 同一时间窗口内积分角速度，两者在角度域做差，消除旋转分量。
 *
 * 与旧方法(V1: LPF 匹配)的关键区别：
 *   V1：光流 → omega(角速度) → 与 LPF 滞后的陀螺仪 omega 做差 → v
 *       问题：光流(过去35ms平均) 与 陀螺仪(此刻滞后20ms) 时间不对齐
 *   V2(本方法)：光流 → angle(总像素角位移) ← 与 ← 陀螺仪(同一窗口积分角) 做差
 *       → trans_angle / dt → omega → v
 *       优势：两个测量值来自同一时间窗口，精确对齐
 *
 * 实现：
 *   ① 2ms PIT 中断里持续积分陀螺仪角速度 → upf_gyro_angle_x/y
 *   ② 10ms 主循环对累积值做快照 → 环形缓冲 GYRO_BUF_SIZE 个条目
 *   ③ 光流新帧到达时，查缓冲得到"过去 timespan_us 内陀螺仪总转角"
 *   ④ 在角度域做差，再除 dt 得到补偿后角速度 */
#define UPF_USE_ANGLE_DOMAIN_COMP   (1)      /* 1=角度域精确补偿(本方法) 0=旧LPF方法 */
#define UPF_GYRO_BUF_SIZE           (80)     /* 环形缓冲条目数(80×2ms=160ms,覆盖最大60ms积分窗)。
                                              * 注意：缓冲区在2ms PIT中断里写入，够大防溢出 */

/* 仰角补偿（倾斜时光轴不再垂直地面）
 * 飞机倾斜时，光流传器光轴不再垂直向下，实际测距沿光轴为 H/cos(pitch)/cos(roll)。
 * 同时水平速度在传感器视角下的投影会变化，但小角度(<12°)下主要误差来自测距，
 * 这里用沿光轴有效距离代替 height_cm 参与速度换算。 */
#define UPF_USE_TILT_COMP           (1)      /* 1=仰角补偿, 0=关 */

/* ==================== 旋转补偿 V1（旧方法，作为角度域补偿的对比/回退） ====================
 *
 * 当 UPF_USE_ANGLE_DOMAIN_COMP = 0 时启用。
 * 原理：光流测的是"地面相对相机的角速度"，但飞机自身有姿态旋转时也会
 * 让相机看到地面在移动——这部分不是真实平移，需要用陀螺仪角速度减掉。
 *
 * V1 问题：光流测量是过去积分窗口(20~50ms)的平均值，而陀螺仪 LPF 滞后
 * 只是粗略匹配，时间没精确对齐。角度域补偿(V2)解决了这个问题。 */
#define UPF_USE_ROTATION_COMP       (1)         /* 主开关：0=关所有旋转补偿 */

/* ---- V1(旧)：LPF匹配法(回退)，仅UPF_USE_ANGLE_DOMAIN_COMP=0时启用 ---- */
#if !UPF_USE_ANGLE_DOMAIN_COMP
#define UPF_GYRO_LPF_HZ             (8.0f)
#define UPF_GYRO_COMP_LIMIT_RAD_S   (1.0f)
#endif

/* ---- V2(新)：角度域法(当前方法) ----
 * 符号与厂商代码一致（角度域做差时使用）：
 *   trans_angle_x = flow_angle_x + gyro_pitch_angle     → X流减 pitch 旋转 ，X_SIGN=+1
 *   trans_angle_y = flow_angle_y - gyro_roll_angle      → Y流减 roll  旋转 ，Y_SIGN=-1
 * 实现见 .c 中 Up_Flow_302_Gyro_Accum_2ms 和 gyro_buf 环形缓冲查询 */
#define UPF_GYRO_COMP_X_SIGN        (1.0f)
#define UPF_GYRO_COMP_Y_SIGN        (-1.0f)
#define UPF_GYRO_COMP_Z_SIGN        (0.0f)      /* 偏航补偿归零 */
/* IMU660RA 陀螺仪原始 LSB → rad/s：raw × 0.061 dps/LSB ÷ 57.2958 = raw × 0.001065 */
#define UPF_GYRO_LSB_TO_RAD_S       (0.001065f)

/* ==================== 控制器约束 ==================== */
#define UPF_ANGLE_LIMIT             (FLOW_MAX_ANGLE_DEG)      /* 修正角限幅（度） */
#define UPF_INT_ERR_GATE_CMS        (120.0f)    /* 积分分离门限 */
#define UPF_CLIMB_VZ_THRESHOLD      (12.0f)     /* Vz超过此值冻结I（爬升时光流有旋转残留） */
#define UPF_I_FREEZE_UNTIL_CM       (90.0f)     /* 40→90：回退。电源线是左右摆动的不确定力，不是恒力。
                                                 * I项对变化的方向会相位滞后→越积越振。
                                                 * 保持冻结到接近目标高度再开，爬升段靠P硬扛、
                                                 * 到顶靠位置环(CAM_POS_KP)和速度环P稳。*/

/* ==================== 阶段1：速度指令 mock + 跟随接口 ====================
 * 双核分工：视觉核检测小车出"相对坐标"，飞控核跟随往目标走。
 * 视觉还没接进来，这里先用 mock 直接给"期望速度"，专门验证：
 *   速度环能不能跟踪一个【非零】设定值（原来恒为 0=悬停）。
 *
 * 接口约定（将来视觉/外环就绪后，只要往 upf_target_vx/vy 写值即可，控制器不变）：
 *   upf_target_vx/vy = 期望对地速度(cm/s)。
 *     0   = 悬停 hold（与原版行为完全一致）
 *     非0 = 让飞机按这个速度走（前进/后退/左右）
 */
/* 分步调试选择器（层层叠加，每步单独飞稳再加下一步）。改这一个宏即可切换：
 *   1 = 仅起飞悬停   —— target 恒 0，纯光流定点。先把这步飞稳。
 *   2 = 速度跟踪     —— 悬停就绪后，mock 发速度指令(前进/后退循环)，验证内环跟踪。
 *   3 = 位置跟随     —— 虚拟车相对坐标→外环→期望速度（阶段2实现后启用）。
 *   4 = 真实摄像头   —— CM7_1视觉检测车灯，IPC发坐标，飞控定点/跟车 */
#define FOLLOW_STEP           (4)   // 真实摄像头定点/跟车
#define CAM_VEL_MOCK_SPEED    (15.0f)    /* 第2步：测试指令速度(cm/s) */
#define CAM_VEL_MOCK_PHASE_S  (3.0f)     /* 第2步：每段持续秒数 */

extern float upf_target_vx;   /* 速度环设定值 X(前后,cm/s)：mock 或相机外环写，Speed_Damp 读 */
extern float upf_target_vy;   /* 速度环设定值 Y(左右,cm/s) */

/* 速度指令 mock：按时间生成 悬停→前进→悬停→后退 的循环指令，写入 upf_target_vx。
 * 主循环在调用 Up_Flow_302_Speed_Damp 之前调用它。@param dt 调用周期(秒)。
 * 接真视觉后，删掉这个调用、改成从共享内存读对方核的坐标即可。*/
void Cam_Vel_Mock_Update(float dt);

/* 复位 mock：把内部计时和目标速度清零（=纯悬停）。
 * 跟车未就绪 / 离开定高时调用，保证下次就绪从"悬停段"重新开始。*/
void Cam_Vel_Mock_Reset(void);

/* ==================== 跟随就绪门限（先爬到目标高度并稳住，再开始跟车）====================
 * 就绪前：upf_target=0，纯悬停定点（只压漂移）。
 * 就绪后：才让 mock/相机 发速度指令去跟车。一旦就绪就锁存，直到离开定高/落地。*/
#define FOLLOW_HEIGHT_BAND_CM  (25.0f)   /* 离 ALT_HOLD_TARGET_CM 多近算"到达目标高度"。15→25：实测稳在~102而目标120(差18)，带太窄会永不锁存→mock不触发 */
#define FOLLOW_VZ_SETTLE_CMS   (8.0f)    /* 垂直速度(cm/s)小于此算"已稳定" */
#define FOLLOW_SETTLE_S        (3.0f)    /* 上面两条同时满足、稳定悬停多久才开始跟车(秒) */

/* ==================== 第3步：虚拟车位置跟随（备用，FOLLOW_STEP=3 启用）====================
 * 数据流：虚拟车世界位置(可静止/走轨迹) → 飞机世界位置(积分光流) →
 *         相对坐标 cam_rel = 车 - 飞机（= 真相机会测到的"车相对飞机"）→
 *         外环P 把相对坐标转成期望速度 upf_target_vx/vy → 喂给速度环（第2步那个环）。
 *
 * 【分离设计】数据源(A) 和 外环(B) 分开：接真相机时只换 A(改成读共享内存+IMU去旋转)，
 *            外环 B 完全不动。*/
#define CAM_POS_KP        (0.30f)   /* 0.75→0.30：和MaplePilot一致，温柔指路不饱和 */
#define V_FOLLOW_MAX      (12.0f)   /* 18→12：降最大期望速度，减少倾斜→减少定高耦合 */
#define VCAR_MODE         (0)       /* 虚拟车运动：0=静止(阶跃测试，先飞这个)  1=正弦往返 */
#define VCAR_STEP_X       (30.0f)   /* 静止模式：车固定在世界系 X=此值(cm)，飞机阶跃飞过去并停住(40→30 留线缆余量) */
#define VCAR_SINE_AMP     (30.0f)   /* 正弦模式：幅度(cm)(50→30 留线缆余量) */
#define VCAR_SINE_PERIOD  (12.0f)   /* 正弦模式：周期(s)(10→12 放慢，跟随更从容) */

/* 接口：车相对飞机坐标(cm)，飞机=原点。mock 或真相机写，外环读。调试可打印观察。*/
extern float cam_rel_x;            /* 车相对飞机 X(前后) */
extern float cam_rel_y;            /* 车相对飞机 Y(左右) */
extern uint8 cam_valid;            /* 1=相对坐标有效，0=丢目标 */
extern uint32 cam_last_update_ms;  /* 视觉最近一次写入时刻(ms) */

/* 调试：相机IPC原始像素/换算cm（台架验符号用）*/
extern int16  cam_dbg_u, cam_dbg_v;
extern int16  cam_dbg_area;
extern uint8  cam_dbg_maxg;
extern float  cam_dbg_x, cam_dbg_y;
extern uint8  cam_dbg_valid;
extern uint16 cam_dbg_rx_cnt;
extern uint32 cam_dbg_last_raw;

/* ==================== 相机接口（真实摄像头，FOLLOW_STEP=4）==================== */
#define CAM_TIMEOUT_MS       (300U)      /* 无新数据超时判丢目标 */
#define CAM_FOCAL_PX         (117.0f)    /* ★焦距像素：2.8mm镜头/6µm像元/4倍下采样=117px */
#define CAM_SWAP_UV          (0)         /* 0:前后←v、左右←u ; 1:互换 */
#define CAM_FWD_SIGN         (-1.0f)     /* +1→-1：实际飞行验证camera锁到后位置环发散，cm_rel_x极性反了。
                                          * 目标在前→px_v负→+1.0×负=负→命令向后飞→越飞越远。
                                          * -1.0×负=正→命令向前飞→收敛。 */
#define CAM_RIGHT_SIGN       (-1.0f)     /* 左右符号 */
#define CAM_ENGAGE_HEIGHT_CM (40.0f)     /* 摄像头接管高度门限 50→40 */
#define CAM_TARGET_HEIGHT_CM (30.0f)     /* 车灯板离地高度(cm) */
#define CAM_MIN_RANGE_CM     (10.0f)     /* 相机→灯板距离下限(cm) */

/* 视觉写入车相对坐标（Cam_IPC_Process 或外部UART调）*/
void Cam_Set_Target(float rel_x_cm, float rel_y_cm, uint8 valid);
/* CM7_0 IPC初始化+轮询读共享信箱 */
void Cam_IPC_Init(void);
void Cam_IPC_Process(float height_cm);

/* (A) 数据源：虚拟车 mock */
void Cam_Pos_Mock_Update(float dt);
/* (B) 跟随外环：cam_rel → 期望速度 upf_target_vx/vy */
void Cam_Follow_Outer_Update(float dt);

/* ==================== C2：起飞/悬停位置保持外环 API ==================== */
void Up_Flow_302_PosHold_Update(float dt);
void Up_Flow_302_PosHold_Reset(void);
void Up_Flow_302_PosHold_Prime(void);    /* 相机接管时钉原点 */

/* ==================== 对外变量（调试/在线观测） ==================== */
extern float  upf_vel_x;            /* 滤波后 X 速度（cm/s），前后向，对应 pitch 修正 */
extern float  upf_vel_y;            /* 滤波后 Y 速度（cm/s），左右向，对应 roll 修正 */
extern uint8  upf_data_valid;       /* 1=速度可信，0=高度超限/帧超时/valid 位不对 */
extern uint8  upf_data_fresh;       /* 1=最近 UPF_VALID_HOLD_MS 内有可信新帧，可继续控制 */
extern float  upf_dbg_raw_vx;       /* 滤波前 X 速度，调试用 */
extern float  upf_dbg_raw_vy;
extern uint16 upf_dbg_frame_cnt;    /* 累计有效帧数 */
extern uint32 upf_dbg_last_frame_ms;/* 上次有效帧到达时刻（ms） */
extern float  upf_dbg_omega_x;      /* 光流角速度（rad/s），旋转补偿后；调试用 */
extern float  upf_dbg_omega_y;
extern float  upf_dbg_gyro_lpf_x;   /* 陀螺仪 LPF 后角速度（rad/s）：roll 通道 */
extern float  upf_dbg_gyro_lpf_y;   /* 陀螺仪 LPF 后角速度（rad/s）：pitch 通道 */
extern float  upf_pitch_corr;        /* optical-flow pitch correction, deg */
extern float  upf_roll_corr;         /* optical-flow roll correction, deg */
extern float  upf_dbg_err_vx;        /* 速度误差 X = target - vel */
extern float  upf_dbg_err_vy;        /* 速度误差 Y */
extern float  upf_dbg_raw_out_pitch; /* 翻 UPF_CTRL_PITCH_SIGN 前的原始输出 */
extern float  upf_dbg_raw_out_roll;  /* 翻 UPF_CTRL_ROLL_SIGN 前的原始输出 */

extern PID_t  pid_upf_vx;           /* X 轴速度阻尼 PI（输出 → pitch 修正） */
extern PID_t  pid_upf_vy;           /* Y 轴速度阻尼 PI（输出 → roll 修正） */

/* ==================== IMU融合变量 ==================== */
extern float  upf_imu_vx;           /* IMU积分速度X（验证用） */
extern float  upf_imu_vy;           /* IMU积分速度Y（验证用） */
extern float  upf_fused_vx;         /* 互补融合后速度X（替代upf_vel_x送入PI） */
extern float  upf_fused_vy;         /* 互补融合后速度Y */
void Up_Flow_302_IMU_Fusion(float dt);  /* 每周期调，算融合速度 */

/* ==================== API ==================== */

/* 在 2ms PIT 中断里调用：累积陀螺仪角位移，供角度域旋转补偿使用。
 * 只在 UPF_USE_ANGLE_DOMAIN_COMP=1 时需要。必须在 main_cm7_0.c 的 pit0_ch0_isr 里调用。 */
void Up_Flow_302_Gyro_Accum_2ms(void);

/* 上电后调用一次。内部调 upflow302_receive_init()（库的函数，按库里宏配置 UART_6 + P03）
 * 还会清空 PI 状态并设默认参数 */
void Up_Flow_302_Init(void);

/* 主循环每 20ms 调用一次。读最新帧、换算物理速度、低通滤波
 * @param height_cm 当前 DL1B 测得的高度（cm） */
void Up_Flow_302_Update(float height_cm);

/* 主循环每 20ms 调用，紧跟在 Update 之后
 * @param dt          调用周期（秒）
 * @param pitch_corr  [out] pitch 修正角（度）
 * @param roll_corr   [out] roll 修正角（度）
 * 注意：pid.c 的 Motor_Control_Mixing 暂未接入这两个输出，等调参再打开 */
void Up_Flow_302_Speed_Damp(float dt, float *pitch_corr, float *roll_corr);

/* 清空速度、滤波器、陀螺 LPF、PI 积分。PDF 1.3.3/3.4 列出的必须清零时机：
 *   - 起飞前 / 自动起飞过程中（flight_state == ALT_TAKEOFF）
 *   - 打摇杆操作时（有手动控制输入）
 *   - 落地 / 急停 / flight_state 回 ALT_IDLE
 *   - 累积位移超出有效范围（如果以后加位置环）
 * 不清的话起飞会斜着飞、松摇杆会反向飘 */
void Up_Flow_302_Reset(void);

#endif /* enable switch */

#endif /* _UP_FLOW_302_H_ */
