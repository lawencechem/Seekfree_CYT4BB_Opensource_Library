#ifndef _OPTICAL_FLOW_H_
#define _OPTICAL_FLOW_H_

#include "zf_common_headfile.h"
#include "pid.h"

#if 0   // PMW3901 optical-flow API disabled: switching to UP FLOW 302.
/*
 * PMW3901 光流速度阻尼模块
 *
 * ==================== 模块定位 ====================
 * 本模块实现"水平速度阻尼"——让无人机在悬停时自动抵消水平漂移。
 * 核心思路：光流传感器测出飞机相对地面的水平移动速度，
 *           如果速度不为零就倾斜机身去抵消，目标是让水平速度趋近于零。
 *
 * ==================== 控制链路（数据流） ====================
 *
 *   PMW3901 硬件                本模块                         姿态控制
 *   ─────────────          ──────────────              ──────────────
 *   delta_x (像素增量)  ──→  速度换算 (cm/s)  ──→  PI 控制器  ──→  pitch/roll 修正角
 *   delta_y (像素增量)  ──→  低通滤波 (去噪)  ──→  速度阻尼    ──→  送给姿态外环
 *                                   ↑
 *                           height (TOF高度)
 *                     （光流速度 = 像素增量 × 高度 / 时间）
 *
 * ==================== 为什么速度换算需要高度？ ====================
 * PMW3901 测的是"像素移动了多少"。同样是 1 个像素的移动：
 *   - 在 10cm 高度 → 飞机只移动了很小的距离
 *   - 在 100cm 高度 → 飞机移动了 10 倍的距离
 * 所以真实物理速度 = 像素增量 × 比例系数 × 当前高度 / 时间间隔
 *
 * ==================== 两种换算方案 ====================
 * 方案A（默认）：经验系数法，简单直接，需要实际标定
 * 方案B：角速度模型法，基于 PMW3901 的光学参数，理论更严谨
 *
 * ==================== 当前开发状态 ====================
 * - 本模块已计算出修正量 (of_pitch_corr_cmd, of_roll_corr_cmd)
 * - 但尚未接入 Motor_Control_Mixing（在 pid.c 中被注释）
 * - 目前仅通过串口打印验证数据正确性
 * - 下一步：验证方向符号正确后，接入姿态外环目标角
 */

/* ==================== 符号标定区 ====================
 *
 * 光流传感器安装方向不同，同一个轴的正方向含义也不同。
 * 这里用 4 个符号宏把"物理安装差异"集中管理，不用到处改代码。
 *
 * 调试方法：
 *   1. 打印 of_vel_x / of_vel_y
 *   2. 手动向前/向后推飞机 → 看 of_vel_x 是否稳定变化，不对就改 OF_VEL_X_SIGN
 *   3. 手动向左/向右推飞机 → 看 of_vel_y 是否稳定变化，不对就改 OF_VEL_Y_SIGN
 *   4. 同理验证 OF_CTRL_PITCH_SIGN / OF_CTRL_ROLL_SIGN
 */

/* 光流速度符号：补偿传感器安装方向导致的坐标翻转
 * 例：传感器倒装时 X/Y 轴都反向，两个都设为 -1 */
#define OF_VEL_X_SIGN              (1.0f)
#define OF_VEL_Y_SIGN              (1.0f)

/* 控制输出符号：补偿"速度→姿态角"映射方向
 * 例：飞机向前飘（of_vel_x>0）应该通过 pitch 修正抵消；如果控制方向反了就改符号 */
#define OF_CTRL_PITCH_SIGN         (-1.0f)
#define OF_CTRL_ROLL_SIGN          (-1.0f)

/* ==================== 有效性门限 ====================
 *
 * 光流传感器只有在特定条件下才能可靠工作：
 * - 太低（<10cm）：镜头对焦模糊，像素增量不可信
 * - 太高（>220cm）：超过 PMW3901 有效量程
 * - 速度超过 300cm/s：超出传感器物理极限，视为噪声
 */
#define OF_MIN_HEIGHT_CM           (10.0f)      // 最低有效高度（PMW3901 最小测距 5cm，留 5cm 余量）
#define OF_MAX_HEIGHT_CM           (220.0f)     // 最高有效高度（PMW3901 推荐上限）
#define OF_MAX_VEL_CMS             (300.0f)     // 单帧速度跳变门限，超过视为噪声

/* ==================== 换算/滤波参数 ==================== */

/* OF_UPDATE_DT_S：光流 Update 函数的调用周期（秒）
 * 外部 main 循环中 of_cnt 每 2 个 10ms 周期调用一次，即 20ms = 0.02s
 * 如果外部调用周期改变，这里必须同步修改，否则速度换算会偏差 */
#define OF_UPDATE_DT_S             (0.02f)

/* OF_VEL_LPF_ALPHA：一阶低通滤波系数（0.0 ~ 1.0）
 * alpha = 0.35 → 新数据占 35%，历史占 65%
 * 越小越平滑（去噪强）但延迟越大（响应慢）
 * 越大响应越快但噪声也更大，需根据实际震动情况调整 */
#define OF_VEL_LPF_ALPHA           (0.35f)

/* 方案A（默认）：经验系数换算
 * OF_COUNT_TO_CMS_PER_CM = 0.0105 含义：
 *   在 1cm 高度下，1 个像素增量对应 0.0105 cm 的物理位移
 *   公式：vel(cm/s) = delta_count × 0.0105 × height_cm / 0.02s
 * 这个系数需要根据实际安装高度和地面纹理做标定 */
#define OF_COUNT_TO_CMS_PER_CM     (0.0105f)

/* 方案B（对照用）：角速度模型换算
 * OF_PMW3901_RAD_PER_COUNT = 0.0244 弧度/count 是 PMW3901 datasheet 给出的
 * 视场角参数，理论上不依赖经验标定
 * 公式：vel(cm/s) = delta_count × 0.0244 × height_cm / dt */
#define OF_PMW3901_RAD_PER_COUNT   (0.0244f)

/* 方案切换开关：0 = 方案A（经验系数），1 = 方案B（角速度模型）
 * 两种方案的结果可以通过串口打印对比验证 */
#define OF_USE_RAD_MODEL           (0)

/* ==================== 控制器约束 ==================== */

/* OF_ANGLE_LIMIT：光流修正角的输出限幅（度）
 * 光流修正叠加到姿态目标角上，这个值限制了"光流最多能倾斜机身多少度"
 * 2.0 度是验证阶段的保守值，实际飞行验证后可放大到 5~8 度 */
#define OF_ANGLE_LIMIT             (0.6f)

/* OF_INT_ERR_GATE_CMS：积分分离门限（cm/s）
 * 当速度误差 > 120 cm/s 时暂停积分，防止大偏差时积分暴走（anti-windup）
 * 当前 ki=0 这个值实际不起作用，开启积分后需要根据响应调整 */
#define OF_INT_ERR_GATE_CMS        (120.0f)

/* ==================== 对外接口 ====================
 *
 * of_vel_x / of_vel_y：滤波后的水平速度（cm/s），供外部观测/调试
 * of_data_valid：数据有效标志，1=有效 0=无效（高度超限或传感器异常）
 * pid_of_vx / pid_of_vy：PI 控制器实例，外部可直接读写参数用于在线调参
 */
extern float of_vel_x;
extern float of_vel_y;
extern uint8 of_data_valid;
extern int16 of_dbg_dx;
extern int16 of_dbg_dy;
extern float of_dbg_raw_vx;
extern float of_dbg_raw_vy;
extern uint16 of_zero_streak;

extern PID_t pid_of_vx;
extern PID_t pid_of_vy;

/* ==================== API 函数 ==================== */

/* Optical_Flow_Init：上电时调用一次，清零所有状态和 PID 参数
 * 在 main() 中 pmw3901_init() 之后调用 */
void Optical_Flow_Init(void);

/* Optical_Flow_Update：每 20ms 调用一次，读取光流数据并换算为速度
 * @param height_cm  当前 TOF 测量高度（cm），用于速度换算
 * 内部会调用 pmw3901_get_motion() 读取传感器原始数据 */
void Optical_Flow_Update(float height_cm);

/* Optical_Flow_Speed_Damp：每 20ms 调用一次，基于速度误差计算姿态修正角
 * @param dt         调用周期（秒），通常传 OF_UPDATE_DT_S
 * @param pitch_corr [out] 输出的俯仰修正角（度）
 * @param roll_corr  [out] 输出的横滚修正角（度）
 * 依赖 Optical_Flow_Update 先执行并更新 of_vel_x/y */
void Optical_Flow_Speed_Damp(float dt, float *pitch_corr, float *roll_corr);

/* Optical_Flow_Reset：落地/急停时调用，清零所有速度和积分状态
 * 防止上次飞行的残留状态影响下次起飞 */
void Optical_Flow_Reset(void);

#endif

#endif
