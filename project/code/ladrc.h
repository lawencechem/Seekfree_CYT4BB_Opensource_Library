#ifndef _LADRC_H_
#define _LADRC_H_

/* ==================== LADRC 三轴角速度观测器 ====================
 * LADRC 始终旁观（不参与控制），输出与 PID 对比用于调参。
 * 当 #define LADRC_ENABLE 1 时，LADRC 替代 PID 控制角速度环。
 *
 * 调参方法：飞完看数据
 *   PO vs LPO → 调 LADRC_P_*（Pitch）
 *   RO vs LRO → 调 LADRC_R_*（Roll）
 *   YO vs LYO → 调 LADRC_Y_*（Yaw）
 * LPO/LRO/LYO 与 PO/RO/YO 越接近说明参数越准。
 * ========================================================= */

/* ── 主开关：0=仅旁观对比, 1=替代PID控制角速度环 ── */
#define LADRC_ENABLE            (0)

/* ==================== 三轴 LADRC 参数 ====================
 * 基于飞行数据对比 PO vs LPO、RO vs LRO、YO vs LYO 整定。
 * WC=10/8时 LADRC 响应≈100ms，与PID的70ms接近，
 * 大幅减少符号相反的情况。
 * ====================================================== */

/* ── Pitch 参数 ── */
#define LADRC_P_WC              (10.0f)  /* 5→10：提速匹配PID响应(14rad/s) */
#define LADRC_P_WO              (30.0f)  /* 15→30：ESO跟踪加快 */
#define LADRC_P_B0              (1.0f)
#define LADRC_P_LIMIT           (1200.0f)

/* ── Roll 参数 ── */
#define LADRC_R_WC              (10.0f)  /* 5→10 */
#define LADRC_R_WO              (30.0f)  /* 15→30 */
#define LADRC_R_B0              (1.0f)
#define LADRC_R_LIMIT           (1200.0f)

/* ── Yaw 参数 ── */
#define LADRC_Y_WC              (5.0f)   /* 2→5 */
#define LADRC_Y_WO              (15.0f)  /* 6→15 */
#define LADRC_Y_B0              (0.7f)
#define LADRC_Y_LIMIT           (400.0f)

/* ── 调试变量 ── */
extern float ladrc_dbg_z1_p, ladrc_dbg_z2_p;  /* Pitch ESO */
extern float ladrc_dbg_z1_r, ladrc_dbg_z2_r;  /* Roll  ESO */
extern float ladrc_dbg_lpo;                    /* LADRC Pitch 输出（对比 PO） */
extern float ladrc_dbg_lro;                    /* LADRC Roll  输出（对比 RO） */
extern float ladrc_dbg_lyo;                    /* LADRC Yaw   输出（对比 YO） */

/* ── API ── */
void ladrc_observe_pitch(float target_rate, float gyro_rate, float dt);
void ladrc_observe_roll(float target_rate, float gyro_rate, float dt);
void ladrc_observe_yaw(float target_rate, float gyro_rate, float dt);
void ladrc_reset(void);

#endif
