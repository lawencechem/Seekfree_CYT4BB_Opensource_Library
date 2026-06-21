---
name: engine-control-system
description: Complete documentation of quadcopter flight control parameters and architecture
metadata:
  type: project
  last-updated: 2026-06-21
---

# Seekfree CYT4BB 飞控系统文档

## 硬件平台
- MCU: Infineon Traveo T2G CYT4BB7CEE (CM7_0=飞控核, CM7_1=视觉核)
- IMU: MPU660RA (陀螺+加速度计)
- 光流: LC-302 (UART6, 20~50Hz)
- 测距: DL1B TOF
- 摄像头: MT9V03X (188×120, 20Hz, 红外灯板检测)
- 系绳: 电源线缆连接车与飞机，产生摆锤扰动(~1Hz)

---

## 控制架构

```
位置环(P+FF+D) → 速度环(P+I) → 角度环(P) → 角速度环(PID/LADRC) → 电机混控
                                                              → 姿态角(欧拉角)
          ↑摄像头cx/cy   ↑光流vx/vy     ↑陀螺姿态    ↑陀螺角速度
```

## 1. 位置环 (Cam_Follow_Outer_Update) — 100Hz

接收摄像头转换后的相对坐标 `cx/cy(cm)`，输出期望水平速度 `upf_target_vx/vy(cm/s)`。

```
err_x =  cam_rel_y                  // 轴映射
err_y = -cam_rel_x
死区: fabs(err) < 4cm → err=0
微分: deriv = (err - last_err) / dt  // 仅帧更新时计算,用真实帧间隔(50ms)除

P:    K=1.28    × err      → 回归力
D:    K=0.15    × d_lpf(deriv) → 速度阻尼(LPF α=0.25)
FF:   K=0.55    × d_lpf(deriv) → 车动预判(LPF α=1.0=直通)

vx_des = P×err + D×deriv + FF
限幅: |vx_des| ≤ V_FOLLOW_MAX=35 cm/s
```

D项特制：deriv只在`cam_rel_x/y`变化>0.01cm时计算→消除摄像头20Hz帧间跳变脉冲。

## 2. 速度环 (Up_Flow_302_Speed_Damp) — 100Hz

接收期望速度，输出pitch/roll修正角(°)给角度环。

```
速度: upf_fused_vx = α×imu_vx + (1-α)×upf_vel_x  (UPF_FUSION_ALPHA=0.15)
X轴:  P=0.50, I=0.00          → 纯P
Y轴:  P=0.64, I=0.03          → 小I消Y轴稳态漂移
Y轴I: i_limit=80, 泄漏×0.99/步(τ≈1s), 冻结至H>90cm
限幅: |输出| ≤ 12° (FLOW_MAX_ANGLE_DEG)
```

## 3. 角度环 (PID_Compute_Angle) — 500Hz

光流修正角叠加到姿态目标，输出目标角速度(°/s)给角速度环。

```
Pitch: KP=4.2, KI=0, out_limit=60°/s
Roll:  KP=4.2, KI=0, out_limit=60°/s
Yaw:   KP=1.8, KI=0, out_limit=55°/s
```

## 4. 角速度环 (rate PID / LADRC) — 500Hz

### PID 模式 (当前，LADRC_ENABLE=0)
```
Pitch rate: KP=14.0, KI=0.04, KD=0.022, i_limit=350, out_limit=1200
Roll rate:  KP=13.5, KI=0.05, KD=0.025, i_limit=300, out_limit=1200
Yaw rate:   KP=10.0, KI=0.10, KD=0, i_limit=100, out_limit=520, YA=85~91(FF)
```
Yaw FF表：已从110~120下调至85~91（YI从+100→-89，前馈偏大），`YAW_FF_ENABLE=1`。
Yaw stall检测：790周期(1.58s)复位。

### LADRC 旁观模式 (RATE_USE_LADRC=1, LADRC_ENABLE=0)
```
Pitch: WC=10, WO=30, B0=1, LIMIT=1200
Roll:  WC=10, WO=30, B0=1, LIMIT=1200
Yaw:   WC=5,  WO=15, B0=0.7, LIMIT=400
```
LADRC当前仅旁观（输出LPO/LRO/LYO对比PO/RO/YO），不参与控制。
ESO anti-windup：饱和时z2×0.995泄放。

## 5. 定高控制 (Altitude_Control_Task) — 100Hz

### TAKEOFF 爬升（纯速度环）
```
软启动: target_Vz 0→8cm/s @ +1.5/s (~5.3s达到)
  pid_alt_vel.kp = 8.0  (爬升段强P)
  pid_alt_vel.ki = 0.16
  I泄漏: ×0.9990/10ms (τ≈10s)
```
先电机预转(1500→3800)→等dbg_takeoff_state=3(FULL)→Vz爬升。
掉高保护: H<60cm时从HOLD退回TAKEOFF重新爬升。

### HOLD 悬停（位置环+速度环串级）
```
切入条件: H > ALT_HOLD_TARGET - 10 = 110-10 = 100cm
位置环:   kp=0.60, out_limit=12cm/s (CLIMB_UP_MAX_SPEED)
速度环:   kp=7.5, ki=0.16, i_limit=120, out_limit=500
I泄漏:   ×0.9990/10ms (τ≈10s)
积分门:   |高度误差|<40cm 且 |速度误差|<80cm/s
目标高度: 110cm, 爬升斜坡12cm/s
```

### ALT_HOLD_PARAMS
```
ALT_HOLD_TARGET_CM = 110cm
CLIMB_UP_MAX_SPEED = 12 (定高段位置环限速)
CLIMB_DOWN_MAX_SPEED = -30
THR_HOVER_DEFAULT = 6900
THR_MIN_OUTPUT = 5200
THR_MAX_OUTPUT = 8300
```

### Vz速度估计
TOF新帧: 差分+LPF(α=0.25)
帧间: 纯衰减×0.97 (不用加速度计—电机振动污染)

### 倾角补偿
`thr_comp = (thr-3200)/cos(pitch)/cos(roll) + 3200`, 限cos>0.5

## 6. 光流旋转补偿

角度域V2补偿（UPF_USE_ANGLE_DOMAIN_COMP=1）：
2ms PIT累积陀螺角位移→环形缓冲80条→新帧时查缓冲得同窗口转角。
`trans_angle = flow_angle + SIGN×gyro_delta` 消除旋转分量。

仰角补偿（UPF_USE_TILT_COMP=1）：`eff_range = H/cos(pitch)/cos(roll)`。

## 7. 摄像头仰角补偿 (Cam_IPC_Process)

```
tilt_u = FOCAL_PX × tan(roll)    // RC
tilt_v = FOCAL_PX × tan(pitch)   // PC
comp_u = u_raw - tilt_u
comp_v = v_raw - tilt_v
```

## 8. 串口调试变量

```
PO/RO/YO — 角速度环PID输出
LPO/LRO/LYO — LADRC旁观输出(对比调参)
PR/RR/YRC — 角度环输出(目标角速度)
PRt/RRt/YR — 实测角速度(陀螺)
PC/RC — 摄像头仰角补偿量(像素)
cx/cy — 位置误差(cm)
tvx/tvy — 位置环输出(目标速度cm/s)
op/or — 速度环输出(°)
evx/evy — 速度环误差(cm/s)
ivx/ivy — 速度环积分
Z1P — LADRC估计角速度(°/s)
Z2P/Z2R — LADRC扰动估计(°/s²)
```

## 9. 已知问题

- 速度环(FLOW_MAX_ANGLE_DEG=12°)饱和→系绳摆锤无法收敛
- Yaw被系绳扭力拽崩→行66后YR暴增到67°/s→发散
- 系绳摆锤的1Hz振荡通过所有环路耦合，纯P/PID控制有90°相位滞后
- LADRC尚未投入使用（还在旁观调参阶段）
