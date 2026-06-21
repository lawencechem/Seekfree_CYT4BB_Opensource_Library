#include "ladrc.h"
#include "zf_common_headfile.h"

/* ==================== LADRC 三轴角速度观测器 ====================
 * LADRC_ENABLE=0：仅旁观，输出供调参对比
 * LADRC_ENABLE=1：替代 PID 控制角速度环
 * ========================================================= */

/* ── 内部结构 ── */
typedef struct {
    float z1;       /* 估计角速度 (deg/s) */
    float z2;       /* 估计角加速度扰动 (deg/s²) */
    float u_sat;    /* 上次饱和输出 */
} ladrc_axis_t;

static ladrc_axis_t ladrc_p = {0,0,0};
static ladrc_axis_t ladrc_r = {0,0,0};
static ladrc_axis_t ladrc_y = {0,0,0};

/* ── 调试变量 ── */
float ladrc_dbg_z1_p = 0, ladrc_dbg_z2_p = 0;
float ladrc_dbg_z1_r = 0, ladrc_dbg_z2_r = 0;
float ladrc_dbg_lpo = 0;
float ladrc_dbg_lro = 0;
float ladrc_dbg_lyo = 0;

/* ── 单轴更新（带 anti-windup）────
 * 地面静止时 target≈0, gyro≈0, z1=0 → e=0 → z2不累积 → 无过冲
 * 饱和时 z2 冻结 → 被外力按住时不会暴走
 */
static float ladrc_run(ladrc_axis_t *s, float target, float y,
                       float dt, float wc, float wo, float b0, float limit)
{
    float u = (wc * (target - s->z1) - s->z2) / b0;
    s->u_sat = f_limit(u, -limit, limit);
    float e = s->z1 - y;
    s->z1 += dt * (s->z2 + b0 * s->u_sat - 2.0f * wo * e);
    if (fabsf(u - s->u_sat) < 1.0f) {
        s->z2 += dt * (-wo * wo * e);
    } else {
        s->z2 *= 0.995f;  /* 饱和时缓慢泄放z2，防止锁死 */
    }
    return s->u_sat;
}

/* ── 对外 API ── */
void ladrc_observe_pitch(float target_rate, float gyro_rate, float dt)
{
    ladrc_dbg_z1_p = ladrc_p.z1;
    ladrc_dbg_z2_p = ladrc_p.z2;
    ladrc_dbg_lpo = ladrc_run(&ladrc_p, target_rate, gyro_rate, dt,
                              LADRC_P_WC, LADRC_P_WO, LADRC_P_B0, LADRC_P_LIMIT);
}

void ladrc_observe_roll(float target_rate, float gyro_rate, float dt)
{
    ladrc_dbg_z1_r = ladrc_r.z1;
    ladrc_dbg_z2_r = ladrc_r.z2;
    ladrc_dbg_lro = ladrc_run(&ladrc_r, target_rate, gyro_rate, dt,
                              LADRC_R_WC, LADRC_R_WO, LADRC_R_B0, LADRC_R_LIMIT);
}

void ladrc_observe_yaw(float target_rate, float gyro_rate, float dt)
{
    ladrc_dbg_lyo = ladrc_run(&ladrc_y, target_rate, gyro_rate, dt,
                              LADRC_Y_WC, LADRC_Y_WO, LADRC_Y_B0, LADRC_Y_LIMIT);
}

void ladrc_reset(void)
{
    ladrc_p.z1 = 0; ladrc_p.z2 = 0; ladrc_p.u_sat = 0;
    ladrc_r.z1 = 0; ladrc_r.z2 = 0; ladrc_r.u_sat = 0;
    ladrc_y.z1 = 0; ladrc_y.z2 = 0; ladrc_y.u_sat = 0;
}
