# CHANGELOG — Seekfree CYT4BB 飞控参数修改记录

> 最后更新: 2026-06-18
> 所有改动已在 `main_cm7_0.c` 的 printf 末尾新增 `PD/RD`（俯仰/横滚 D 项贡献）调试输出

---

## 一、姿态环（pid.c）—— 已恢复原始纯P

### 一键切换宏（新增）
```c
#define ATTITUDE_STYLE_MAPLEPID    (0)    // 0=经典纯P, 1=成品PD+I(未调好)
```
位置：`pid.c` 第 317 行。改成 1 即切换到成品 PD+I 参数，但目前不稳定，建议保持 0。

### 当前生效参数（纯P风格）

| 轴 | KP | KI | KD | D_LPF_alpha | 输出限幅 |
|-----|----|----|----|-------------|---------|
| 俯仰率 | 14.0 | 0.04 | 0.022 | 0.15 | ±1200 |
| 横滚率 | 13.5 | 0.05 | 0.025 | 0.15 | ±1200 |
| 偏航率 | 10.0 | 0.10 | 0.0 | 1.0 | ±520 |
| 俯仰角 | 4.2 | 0 | 0 | — | ±60°/s |
| 横滚角 | 4.2 | 0 | 0 | — | ±60°/s |
| 偏航角 | 1.8 | 0 | 0 | — | ±55°/s |

### 新增：Yaw 故障检测（借鉴 MaplePilot/无名飞控）
```
位置：pid.c PID_Compute_Yaw_Rate() 之后
逻辑：|y_out| > 一半限幅(260) 且 |y_rate_fb| < 30°/s 持续2秒
      → pid_yaw_rate.integral = 0; target_yaw = current_euler.yaw;
```
防电源线缠绕时 YI 无限积累。

---

## 二、定高环（dl1b_altitude.c/.h）—— 已恢复原始值

| 参数 | 当前值 | 说明 |
|------|-------|------|
| `pid_alt_pos.kp` | 0.60 | 高度位置环 P |
| `CLIMB_UP_MAX_SPEED` | 12.0 cm/s | 最大爬升速度 |
| `CLIMB_DOWN_MAX_SPEED` | -30.0 cm/s | 最大下降速度 |
| `ALT_HOLD_TARGET_CM` | 120.0 cm | 目标悬停高度 |

---

## 三、光流/速度环（UP_FLOW_302.h/.c）—— 已恢复原始值

| 参数 | 当前值 | 说明 |
|------|-------|------|
| `FLOW_VX_KP` | 0.70 | 速度环 X 轴 P |
| `FLOW_VY_KP` | 0.84 | 速度环 Y 轴 P |
| `FLOW_VX_KI` | 0.04 | 速度环 X 轴 I |
| `FLOW_VY_KI` | 0.06 | 速度环 Y 轴 I |
| `FLOW_VX_KD` | 0.0 | 速度环 X 轴 D（关）|
| `FLOW_VY_KD` | 0.0 | 速度环 Y 轴 D（关）|
| `FLOW_MAX_ANGLE_DEG` | 12.0° | 速度环最大修正角 |
| `UPF_I_FREEZE_UNTIL_CM` | 90.0 cm | I 项解冻高度 |
| `FLOW_VEL_LPF_ALPHA` | 0.15 | 光流速度低通 |

---

## 四、摄像头位置环（UP_FLOW_302.h + main_cm7_0.c）—— 已恢复原始值

| 参数 | 当前值 | 说明 |
|------|-------|------|
| `CAM_POS_KP` | 2.0 | 位置环 P |
| `V_FOLLOW_MAX` | 18.0 cm/s | 最大期望速度 |
| `CAM_ENGAGE_HEIGHT_CM` | 40.0 cm | 摄像头介入高度 |
| `CAM_FWD_SIGN` | -1.0 | 摄像头前后方向 |
| `CAM_RIGHT_SIGN` | -1.0 | 摄像头左右方向 |

### 介入逻辑（main_cm7_0.c FOLLOW_STEP=4）
```c
if (cam_valid && H >= 40cm && |Vz| < 12) {
    位置环介入
} else {
    upf_target_vx/vy = 0  // 光流只阻尼
}
```
- `cam_latched` 一次性锁存机制已移除（cv=0 从未触发过）
- Vz<12 门限恢复（防爬升倾斜耦合）

---

## 五、Yaw（pid.c）—— 已恢复原始值

| 参数 | 当前值 |
|------|-------|
| `YAW_FF_ENABLE` | 0（前馈关）|
| `YAW_HEADING_HOLD` | 1（航向锁定开）|
| `pid_yaw_angle.kp` | 1.8 |
| `pid_yaw_rate.kp` | 10.0 |
| `pid_yaw_rate.ki` | 0.10 |

---

## 六、D 项调试打印（main_cm7_0.c）

printf 末尾新增：
```
PD:%.2f RD:%.2f
```
- PD = `pid_pitch_rate.kd × pid_pitch_rate.derivative`（俯仰 D 贡献）
- RD = `pid_roll_rate.kd × pid_roll_rate.derivative`（横滚 D 贡献）

观察方法：
- PD/RD ≈ 0 → D 没起作用
- PD/RD 高频抖动 → D 放大噪声，需降 `d_lpf_alpha` 或 `KD`
- PD/RD 平缓变化 → D 正常阻尼

---

## 七、各模块改动清单

| 模块 | 文件 | 改动 |
|------|------|------|
| 姿态PID | `pid.c` | 恢复原始纯P参数；新增`ATTITUDE_STYLE_MAPLEPID`宏；新增yaw故障检测 |
| 定高 | `dl1b_altitude.c` | `pid_alt_pos.kp` 1.0→0.6 |
| 定高 | `dl1b_altitude.h` | `CLIMB_UP_MAX_SPEED` 8→12 |
| 光流/速度环 | `UP_FLOW_302.h` | `I_FREEZE` 30→90, `ENGAGE_HEIGHT` 100→40 |
| 主循环 | `main_cm7_0.c` | 移除`cam_latched`，恢复Vz门限；新增PD/RD打印 |
