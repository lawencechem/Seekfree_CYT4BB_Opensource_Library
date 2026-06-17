# Seekfree CYT4BB 四轴飞行控制器

基于逐飞 CYT4BB 平台 (Infineon Traveo T2G CYT4BB7CEE 双核 MCU) + IMU660RA 惯性测量单元 + LC-302 光流定点 + MT9V03X 红外摄像头的自主悬停/跟车四轴飞行控制器。

---

## 目录

- [硬件平台](#硬件平台)
- [系统架构](#系统架构)
- [文件结构](#文件结构)
- [控制链路详解](#控制链路详解)
  - [姿态控制 (PID)](#姿态控制-pid)
  - [定高控制 (DL1B TOF)](#定高控制-dl1b-tof)
  - [光流速度环 (LC-302)](#光流速度环-lc-302)
  - [双核视觉定位 (MT9V03X + CM7_1)](#双核视觉定位-mt9v03x--cm7_1)
  - [位置环与跟车](#位置环与跟车)
  - [起飞保护状态机](#起飞保护状态机)
- [关键调试历程与对比](#关键调试历程与对比)
  - [旋转补偿的演进](#旋转补偿的演进)
  - [IMU-光流融合为何最终关闭](#imu-光流融合为何最终关闭)
  - [速度环 PI 参数整定](#速度环-pi-参数整定)
  - [YAW 锁定优化](#yaw-锁定优化)
  - [爬升段稳定性](#爬升段稳定性)
  - [LC-302 vs PMW3901 对比](#lc-302-vs-pmw3901-对比)
- [串口调试变量手册](#串口调试变量手册)
- [调试方法论](#调试方法论)
- [代码规范与设计哲学](#代码规范与设计哲学)

---

## 硬件平台

| 组件 | 型号 | 接口 | 说明 |
|------|------|------|------|
| 主控 | CYT4BB7CEE (TVIIBH4M) | 双核 ARM Cortex-M7 @ 250MHz | CM7_0 飞控 + CM7_1 视觉 |
| IMU | IMU660RA (BMI088 兼容) | I2C | 6 轴 (加速度计 ±24g + 陀螺仪 ±2000°/s) |
| 光流 | LC-302 (UP FLOW 302) | UART6 @ 19200bps | 14B 自定义协议，像素积分输出，20~50Hz |
| 测距 | DL1B TOF | I2C | 激光测距，量程 ~10m，精度 ±1cm |
| 摄像头 | MT9V03X | DVP + FIFO | 120×188 灰度图像，红外灯板 940nm 信标 |
| 电机 | RS2205 2300KV | 无刷 | X 型四轴，20A BLHeli_S 电调 |
| 电池 | 3S LiPo 2200mAh | 11.1V | ADC 分压监测，油门补偿 |

### 引脚分配

| 外设 | 引脚 | 备注 |
|------|------|------|
| LC-302 UART | P03.0 (RX), P03.1 (TX) | UART6，与 SPI3 资源冲突但未使用 |
| DL1B I2C | I2C 总线 | 与 IMU660RA 共用 |
| MT9V03X | DVP 并行 + FIFO | CM7_1 专用 |

---

## 系统架构

### 控制层级（由内到外）

```
┌─────────────────────────────────────────────────┐
│        高度环 (DL1B TOF) — 10ms                  │
│  ALT_HOLD → PID 高度/速度 → throttle_output     │
├─────────────────────────────────────────────────┤
│   速度环 (LC-302 光流) — 10ms ← 爬升时关闭      │
│  target=0 → PI(D) → pitch/roll 修正角           │
│  FA 门控: Vz<50 && upf_data_fresh && H>10       │
├─────────────────────────────────────────────────┤
│        姿态角环 (2ms PIT 定时器)                  │
│  pitch/roll_target → PID_Compute_Angle →        │
│  → 目标角速度 → PID_Compute_Rate → 混控输出      │
├─────────────────────────────────────────────────┤
│             电机混控 (X型)                        │
│  M1=RF, M2=LF, M3=RR, M4=LR                    │
└─────────────────────────────────────────────────┘
```

### 双核分工

```
CM7_0 (飞控核)                        CM7_1 (视觉核)
─────────────                        ─────────────
PIT_CH0 2ms 中断:                     MT9V03X 帧完成:
  IMU读 → 滤波 → 姿态解算              阈值化 → 求亮点质心
  PID_Compute → 混控 → PWM 输出          → cam_share_write()
                                        ↓
主循环 10ms:                          CAM_SHARE 信箱 @ 0x280A0000
  DL1B 定高                             (两核关 D-Cache, 天然一致)
  LC-302 光流速度阻尼
  Cam_IPC_Process() ← 轮询读信箱
```

### 数据流

```
姿态环(2ms PIT)              主循环(10ms)
─────────────────          ─────────────────
IMU660RA (I2C)              DL1B TOF (I2C)
  ↓ last_gyro/acc             ↓ height_cm
 Update_Gyro_Filter          Altitude_Control_Task
 Update_Accel_KF               ↓ throttle_output
  ↓ filtered_data            Cam_IPC_Process (CM7_1视觉)
 Common_IMU_GetEuler           ↓ cam_rel_x/y
  ↓ current_euler             LC-302 (UART)
 Motor_Control_Mixing           ↓ upf_vel_x/y
  ↓ M1~M4 PWM                 Up_Flow_302_Speed_Damp
                                ↓ upf_pitch/roll_corr
                               → 叠加到 pitch/roll_target
```

---

## 文件结构

### 应用层 (project/ 目录)

| 文件 | 职责 | 行数 |
|------|------|------|
| [user/main_cm7_0.c](project/user/main_cm7_0.c) | CM7_0 主循环：10ms 定高/光流/视觉调度，串口打印 | ~350 |
| [user/main_cm7_1.c](project/user/main_cm7_1.c) | CM7_1 主循环：红外灯检测 + IPC 写入 | ~70 |
| [code/pid.c](project/code/pid.c) | PID 控制器、姿态混控、X 型混控矩阵、起飞保护状态机、Yaw 前馈表 | ~800 |
| [code/pid.h](project/code/pid.h) | PID 参数结构体、全局变量声明 | ~76 |
| [code/UP_FLOW_302.c](project/code/UP_FLOW_302.c) | LC-302 应用层：数据解析、旋转补偿、互补融合、速度 PI、位置保持、相机接口 | ~855 |
| [code/UP_FLOW_302.h](project/code/UP_FLOW_302.h) | 光流模块所有宏定义、API 声明、调试变量、调参集中管理 | ~338 |
| [code/quaternion.c](project/code/quaternion.c) | Madgwick 姿态解算 (IMU660RA 专用符号适配) | — |
| [code/mpu660ra.c](project/code/mpu660ra.c) | IMU660RA 驱动适配层、零点校准 | — |
| [code/filter.c](project/code/filter.c) | 陀螺仪一阶 LPF、加速度计卡尔曼滤波 | ~30 |
| [code/filter.h](project/code/filter.h) | 卡尔曼滤波器结构体 | ~30 |
| [code/dl1b_altitude.c](project/code/dl1b_altitude.c) | DL1B 定高：位置环/速度环 PID、倾角油门补偿 | — |
| [code/dl1b_altitude.h](project/code/dl1b_altitude.h) | 飞行状态机枚举、定高 PID 参数、安全保护 | ~74 |
| [code/battery_comp.c](project/code/battery_comp.c) | 电池电压采样、低通滤波、油门补偿 | — |
| [code/battery_comp.h](project/code/battery_comp.h) | 补偿参数：LPF α=0.15, 标称电压 11.2V | ~45 |
| [code/camera.c](project/code/camera.c) | MT9V03X 阈值化 + 质心计算（无 BFS/连通域，O(n)） | — |
| [code/camera.h](project/code/camera.h) | 摄像头参数：阈值 60, ROI 半宽 30, 面积门限 4~6000 | ~42 |
| [code/cam_share.h](project/code/cam_share.h) | 双核共享信箱 @ 0x280A0000（inline 读写） | ~52 |
| [code/small_driver_uart_control.c](project/code/small_driver_uart_control.c) | 电调 UART 控制协议 (PWM 替代) | — |

### 逐飞底层库 (libraries/ 目录，仅列出涉及改动的)

| 文件 | 改动 |
|------|------|
| zf_device/zf_device_upflow302.h | UART_6 + P03.0/P03.1（GBK 编码） |
| zf_device/zf_device_upflow302.c | 未改动 |

---

## 控制链路详解

### 姿态控制 (PID)

#### 控制流程（每 2ms 执行）

```
PIT_CH0 中断:
  IMU660RA 读原始数据 (gyro+acc)
  Update_Gyro_Filter → 一阶 LPF
  Update_Accel_KF → 卡尔曼滤波
  Common_IMU_GetEulerAngle → Madgwick 解算 → current_euler {pitch, roll, yaw}
  Motor_Control_Mixing → 混控输出 → small_driver_set_duty()

Motor_Control_Mixing 内部:
  ① 起飞保护状态机判断 (见后)
  ② pitch_target = zero_offset + flight_trim + upf_pitch_corr (光流修正)
     roll_target  = zero_offset + flight_trim + upf_roll_corr
  ③ PID_Compute_Angle(角度外环) → 目标角速度 p_target_rate
  ④ PID_Compute_Rate(角速度内环) → p_out, r_out
  ⑤ PID_Compute_Yaw_Rate(偏航专用内环) → y_out + yaw_bias_for_mix → y_limited
  ⑥ 动力余量保护 Attitude_Mix_Headroom_Limit
  ⑦ X 型混控矩阵 → M1~M4 PWM
```

#### PID 参数

| 通道 | 环 | Kp | Ki | Kd | out_limit | 说明 |
|------|------|----|----|----|-----------|------|
| Pitch 角 | 角度环 | 4.2 | 0 | 0 | 25 deg/s | 只 P，输出目标角速度 |
| Pitch 率 | 角速度环 | 14.0 | 0.04 | 0.022 | 1200 | 内环阻尼，D_LPF=0.15 |
| Roll 角 | 角度环 | 4.2 | 0 | 0 | 25 deg/s | 只 P |
| Roll 率 | 角速度环 | 13.5 | 0.05 | 0.025 | 1200 | 内环阻尼 |
| Yaw 角 | 角度环 | 1.8 | 0 | 0 | 55 deg/s | 航向保持，输出目标偏航角速度 |
| Yaw 率 | 角速度环 | 10.0 | 0.10 | 0 | 520 | 偏航阻尼，anti-windup |

**调试历程对比：**

| 参数 | 初始值 | 中间值 | 当前值 | 变化原因 |
|------|--------|--------|--------|----------|
| pitch_rate.kp | 8.0 | 7.0→9.4→11→14 | 14.0 | 逐步加强，飞机更硬、响应更快 |
| pitch_rate.kd | 0.05 | 0.008→0.035→0.022 | 0.022 | D太大→高频振荡；太小→侧翻 |
| roll_rate.kp | 8.0 | 9.4→10.5→13.0 | 13.5 | 与 pitch 对称但略低 |
| roll_rate.kd | 0.05 | 0.008→0.035→0.04 | 0.025 | 同上 |
| yaw_angle.out_limit | 80 | — | 55 | 减少外环过冲防振荡 |
| yaw_angle.kp | 1.12 | — | 1.8 | 加强 yaw 锁定 |

#### X型混控

```
M1 (RF) = base + p_out + r_out + y_mix     // 右前
M2 (LF) = base + p_out - r_out - y_mix     // 左前
M3 (RR) = base - p_out + r_out - y_mix     // 右后
M4 (LR) = base - p_out - r_out + y_mix     // 左后
```

注意：quaternion.c 中对 gyro_x/y 做了符号取反，+r_out 产生左倾。混控符号已适配。

#### Yaw 油门前馈

反扭矩前馈表：7 个油门断点 (6000~7200)，对应 7 个前馈值。

```
thr_bp:  6000  6200  6400  6600  6800  7000  7200
bias:    110   112   114   116   117   118   120
```

- **前馈关闭** (`YAW_FF_ENABLE = 0`)：诊断模式，纯靠 PID 纠偏
- 自学习已关闭，需通过串口日志中 YI 手动调表
- 快速低通 `α = 0.03`，避免油门突变时前馈跳变
- 动力余量保护：优先保 pitch/roll（保命），多出来的余量才分给 yaw

### 定高控制 (DL1B TOF)

#### 控制架构

```
10ms 主循环:
  dl1b_get_distance() → TOF 测距
  Altitude_Control_Task(dt, tof_has_new)
    └─ PID 高度外环 → 目标垂直速度 (CLIMB_UP_MAX=12, CLIMB_DOWN_MAX=-30 cm/s)
    └─ PID 垂直速度内环 → throttle 修正量
    └─ + 悬停基值 (THR_HOVER_DEFAULT=6900) → throttle_output
    └─ 倾角补偿 (Tilt_Compensate: 倾斜时分摊部分油门到姿态，保持总升力)
    └─ 电池电压补偿 (Battery_Comp_Apply: scale = V_nominal / V_batt)
```

| 参数 | 值 | 说明 |
|------|-----|------|
| THR_HOVER_DEFAULT | 6900 | 悬停油门基值 (3S 2200mAh) |
| THR_MAX_OUTPUT | 8300 | 公共油门上限 |
| THR_MIN_OUTPUT | 5200 | 公共油门下限 |
| CLIMB_UP_MAX_SPEED | 12 cm/s | 爬升限速，保证光流全程有效 |
| ALT_HOLD_TARGET_CM | 120 cm | 定高目标高度 |

### 光流速度环 (LC-302)

#### 分层架构

```
逐飞库层 (zf_device_upflow302):
  UART6 ISR → 逐字节收帧 → XOR 校验 → upflow302_receive 结构体
  (已手动把 UART 引脚改为 P03.0/P03.1)

应用层 (UP_FLOW_302.c):
  Up_Flow_302_Update(height):
    ① 高度门控 (10~300cm)
    ② 检测新帧 (upflow302_finsh_flag)
    ③ 互换 X/Y（模块安装方向与飞控轴向差 90°）
    ④ 物理速度换算: v = (pixel / 10000) / dt_s × height_cm × SCALE(0.5)
    ⑤ 跳变剔除 (>80cm/s 用上次滤波值替换)
    ⑥ 旋转补偿 (陀螺仪 LPF 8Hz 减掉姿态旋转分量)
    ⑦ 一阶 LPF (α=0.15)
    ⑧ 有效性状态输出 (valid + fresh)

  Up_Flow_302_Speed_Damp(dt):
    ① 非 fresh → 输出 0 退出
    ② 速度误差 = target - vel
    ③ 死区 (2cm/s)
    ④ 积分冻结 (|Vz|>12 不积，防爬升 windup)
    ⑤ 泄漏积分 (×0.99/步，约 2s 时间常数)
    ⑥ P + I + D(低通α=0.25) → out_pitch/out_roll
    ⑦ 符号校准 → pitch_corr/roll_corr
```

#### 速度环参数

| 宏 | 值 | 说明 |
|----|-----|------|
| FLOW_VX_KP | 0.35 | X轴(前后)比例增益 |
| FLOW_VY_KP | 0.42 | Y轴(左右)比例增益 |
| FLOW_VX_KI | 0.04 | X轴积分 |
| FLOW_VY_KI | 0.06 | Y轴积分 |
| FLOW_VX_KD | 0.02 | X轴微分 |
| FLOW_VY_KD | 0.02 | Y轴微分 |
| FLOW_VEL_LPF_ALPHA | 0.15 | 速度低通 |
| FLOW_VEL_SCALE | 0.5 | 物理速度换算系数 (empirical) |
| FLOW_VEL_DEADBAND_CM_S | 2.0 | 死区 |
| FLOW_MAX_ANGLE_DEG | 12.0 | 修正角限幅 |
| i_limit | 8.0 | 积分限幅 (Init 中设置) |

**参数演进对比：**

| 参数 | V1 | V2 | V3 | V4 | V5 | 最终 |
|------|----|----|----|----|----|------|
| VX_KP | 0.15 | 0.25 | 0.30 | 0.35 | 0.40→振荡 | **0.35** |
| VY_KP | 0.15 | 0.25 | 0.30 | 0.38 | 0.45→振荡 | **0.42** |
| VX_KI | 0.08 | 0.15 | 0.20→振荡 | — | — | **0.04** |
| VY_KI | 0.15 | 0.30→振荡 | — | — | — | **0.06** |
| KD | 0 | 0.05→振荡 | — | — | — | **0.02** |
| LPF_ALPHA | 0.35 | — | — | — | — | **0.15** |
| MAX_ANGLE | 5° | 8° | 10° | 14° | 12° | **12°** |

#### 速度环 PI 极性

| 宏 | 值 | 调试历程 |
|----|-----|----------|
| UPF_VEL_X_SIGN | -1.0 | 初始+1→-1（互换 X/Y 后翻号，向前=正） |
| UPF_VEL_Y_SIGN | +1.0 | 未变（向右=正） |
| UPF_CTRL_PITCH_SIGN | -1.0 | 保持-1 |
| UPF_CTRL_ROLL_SIGN | -1.0 | -1→+1→-1（最终实测 rc 一直≥0 符号反，改回-1） |

#### flow_active 门控

```
flow_active = (H > 10cm && upf_data_fresh && |Vz| < 50cm/s)
```

- 悬停时：FA=1，速度环全开
- 爬升中（Vz>50）：FA=0，速度环关闭 → 纯姿态定高
- **积分冻结**：|Vz| > 12 时 PI 积分停止累积（P/D 仍在工作）
- **切入平滑**：corr_engage 从 0→1 渐变 ~500ms，防瞬态冲击
- **flow_weight**：高度 10~15cm 线性渐入 (0→1)

### 旋转补偿

#### 原理

光流传感器测量的是"地面相对相机的角速度"，包含真实平移 + 姿态旋转。
必须用陀螺仪角速度减掉旋转分量，才能得到纯平移速度。

#### 公式

```
omega_x_corr = omega_x + gyro_lpf_y × UPF_GYRO_COMP_X_SIGN
             = omega_x + 1.0 × gyro_lpf_y      // X 流补偿 pitch 角速度

omega_y_corr = omega_y + gyro_lpf_x × UPF_GYRO_COMP_Y_SIGN
             = omega_y - 1.0 × gyro_lpf_x      // Y 流补偿 roll 角速度
```

#### 调试历程对比

| 版本 | 截止频率 | 符号 | 相位滞后 | 效果 |
|------|---------|------|---------|------|
| 关补偿 | N/A | N/A | N/A | pitch摇晃产生±90cm/s假速度，不可飞 |
| 3Hz + filter.c | 3Hz | 厂商参考 | 53ms | 悬停改善，爬升时残留大 |
| 8Hz + filter.c | 8Hz | 厂商参考 | 40ms | 串级LPF等效5Hz，仍不够 |
| 8Hz + last_gyro | 8Hz | 厂商参考→正负反 | 20ms | 避开串级，效果显著 |
| 8Hz + 修正符号 | 8Hz | X=+1, Y=-1 | 20ms | ✅ 当前最佳 |

关键发现：
- 不要用 filter.c 的 LPF 输出（已经滤过一次），直接用 last_gyro（原始值 + 零偏补偿）
- 否则与 upf_lpf1 形成串级低通，等效截止频率约 5Hz，滞后严重
- gyro_z（偏航）不应参与补偿：纯偏航旋转不产生 X/Y 方向净光流

### 双核视觉定位 (MT9V03X + CM7_1)

#### 检测原理

- 车上安装 940nm 红外灯板，摄像头镜头加 920nm 长通滤光片
- 画面里基本只剩灯板一个亮点
- 固定高阈值 (60) 二值化 → 求亮点质心（不连通域、不 BFS、大数组）
- 质心相对图像中心偏移 = "灯板相对飞机"的方向

```
CM7_1 每帧:
  camera_process()
    └─ ROI 跟踪：锁定后在上一帧质心附近 ROI_HALF=30 窗口内搜索
    └─ 阈值化 → 求质心 (cam_error_x, cam_error_y)
    └─ 面积门限 (4~6000) 判噪点/异常
  cam_share_write(px_u, px_v, valid, area, maxg)
    └─ 信箱 @ 0x280A0000（CM7_1 SRAM）

CM7_0 每 10ms:
  Cam_IPC_Process(height)
    └─ 轮询 CAM_SHARE->seq 判断新帧
    └─ 换算: fwd = CAM_FWD_SIGN * px_fwd * (H - CAM_TARGET_HEIGHT) / CAM_FOCAL_PX
    └─                    = 1.0 * px_fwd * (H - 30) / 117
    └─ Cam_Set_Target(fwd_cm, right_cm, valid)
```

#### IPC 共享信箱协议

| 字段 | 类型 | 说明 |
|------|------|------|
| seq | uint32 | 序列号，每写+1，判新帧 |
| px_u | int16 | 水平像素偏移 |
| px_v | int16 | 垂直像素偏移 |
| area | int16 | 本帧亮点数 |
| valid | uint8 | 1=锁到 |
| maxg | uint8 | 全图最大灰度 |

两核 DCache 均已关闭 → SRAM 天然一致性，无锁无死锁。

#### 参数

| 宏 | 值 | 说明 |
|----|-----|------|
| CAM_FOCAL_PX | 117 | 2.8mm / 6µm / 4×downsample = 117px |
| LED_THRESHOLD | 60 | 红外灯检测阈值（无滤光片需配合maxg精调） |
| ROI_HALF | 30 | 锁定后跟踪窗口半宽 |
| CAM_FWD_SIGN | 1.0 | 前后符号（已台架验证） |
| CAM_RIGHT_SIGN | -1.0 | 左右符号 |
| CAM_ENGAGE_HEIGHT_CM | 40 | 摄像头接管高度门限 |
| CAM_TARGET_HEIGHT_CM | 30 | 灯板离地高度 |

#### 抗干扰三重防护

1. **硬件主防**：滤光片把 850nm 红外信标挡在门外，只有 940nm 灯板可见
2. **ROI 跟踪**：锁定后只在上一帧附近找，外来信标在窗口外自动排除
3. **面积门限**：太小=噪点、太大=异常/反光，都判丢目标

### 位置环与跟车

#### 分层设计

```
数据源 (A)                  外环 (B)                  内环 (C)
─────────                  ───────                  ───────
Cam_Pos_Mock_Update()      Cam_Follow_Outer_Update  Up_Flow_302_Speed_Damp
  └─ 积分光流=飞机位置         └─ cam_rel → vx/vy      └─ PI 速度阻尼
  └─ 车位置(静止/正弦)         └─ P: KP=0 (关)         └─ pitch/roll_corr
  └─ 相对坐标 = 车-飞机         └─ 限幅 ±18cm/s
                        ↑ 真相机就绪只换 A，B 和 C 不改
```

#### 分步调试

| 步骤 | 宏 | 行为 |
|------|-----|------|
| 1 | `FOLLOW_STEP=1` | 起飞/悬停位置保持（目标=起飞点原点），速度和位置 P 都关 |
| 2 | `FOLLOW_STEP=2` | 速度指令 mock：悬停→+15cm/s→悬停→-15cm/s 循环 |
| 3 | `FOLLOW_STEP=3` | 虚拟车位置跟随：模拟相对坐标→外环P→速度指令 |
| 4 | `FOLLOW_STEP=4` | 真实摄像头定点/跟车 |

**跟随就绪门控**：必须先爬到目标高度 (±25cm) + 垂直速度 <8cm/s + 稳定 3s，
才切换到跟车模式。防止爬升中光流污染导致跟车指令误触发。

### 起飞保护状态机

```
ALT_IDLE ──→ TAKEOFF (油门>400)
               │
           PRESPIN (600ms)
               │ 四个电机同步预转 MOTOR_IDLE_DUTY=3500
               │ 不叠加任何姿态修正，清 PID 历史
               │
           RAMP_LIMIT (1200ms)
               │ 油门线性渐增，trim 0→1 渐入
               │ P/R/Y 输出限幅 (250/250/120)
               │ 倾角超过 30° → 急停锁存
               │
           FULL
               │ 完整姿态 + 定高 + 光流控制
               │ 坠机检测：P/R > 70° 持续 35帧→急停
```

**关键设计**：
- Yaw_i_enable：H<50cm 关闭，防止起飞段积分提前推到一侧
- trim 高度渐入：TRIM_ENABLE=8cm ~ TRIM_FULL=30cm
- 中止锁存：一旦触发必须回到 ALT_IDLE 或油门<400 才能重新起飞

---

## 关键调试历程与对比

### 旋转补偿的演进

整个项目中最关键的调试历程。从"纯光流不可飞"到"旋转补偿 + 光流速度环稳定"经历了 4 轮迭代。

| 阶段 | 描述 | 问题 | 解决 |
|------|------|------|------|
| 0 | 纯光流，不开补偿 | pitch 晃动→±90cm/s 假速度 | 必须开补偿 |
| 1 | 3Hz LPF + filter.c 输出 | 相位滞后 53ms，爬升滞后更严重 | 提频到 8Hz |
| 2 | 8Hz LPF + filter.c 输出 | 串级 LPF 等效 5Hz | 改用 last_gyro 避开滤 |
| 3 | 8Hz + last_gyro + 反符号 | 旋转时速度变大 | 对比厂商代码，符号反了 |
| 4 | 8Hz + last_gyro + 厂商符号 | ✅ 角度环跟踪从 30% 提升到 90% | 当前稳定状态 |

**核心教训**：同一份数据经过两级 LPF 会造成等效截止频率远低于单级，相位滞后叠加是灾难性的。

### IMU-光流融合为何最终关闭

| 尝试 | 现象 | 原因 |
|------|------|------|
| 纯 IMU 积分 | 10 秒漂移到 100 cm/s | 加速度零偏 + 重力分离误差 |
| 互补融合 α=0.15 | 融合速度被 IMU 缓慢拉偏 | IMU 漂移虽慢但持续，最终污染光流 |
| 加爬升检测 (|Vz|>12) | 爬升段避免，但悬停段仍漂 | 悬停时段够长就漂飞 |
| 加 IMU 量级缩放 /4 | 漂移降为 1/4，仍不可接受 | 比值不稳定，1×~5× 随机变化 |
| 改进重力分离 | 仍然有 0.9cm/s/秒 漂移 | 本质是 IMU 不可靠 |

**最终结论**：`#define UPF_USE_IMU_FUSION (0)`。纯光流 + 旋转补偿 + LPF 在高 >10cm 时足够用。

### 速度环 PI 参数整定

#### P 的整定

| Kp | 行为 | 结论 |
|-----|------|------|
| 0.15 | 晃动慢，收敛慢 | 太小 |
| 0.25 | 改善但仍慢 | 略小 |
| 0.30 | 较好，轻微过冲 | 接近边界 |
| 0.35 | 稳定边界，响应快 | ✅ VX 最佳 |
| 0.40 | 高频抖动发散 | 过大 |
| 0.45 | 炸机 | 过大 |

Y 轴（Roll 方向）机械响应比 X 轴略差，VY_KP 需要略高 (0.42 vs 0.35)。

#### I 的整定

| Ki | 现象 | 结论 |
|----|------|------|
| 0.08 (VX) | 爬升时积分 windup，恢复后反转 | I 必须小 |
| 0.15 (VX) | 恒力补偿好，但过冲大 | 太大 |
| 0.04 (VX) | ✅ 均衡 | 最佳 |
| 0.06 (VY) | ✅ 比 VX 略大，补偿机械不对称 | 最佳 |

**泄漏积分** `integral *= 0.99f`：约 2s 时间常数，只记近期速度，防随机游走累积。

#### D 的整定

| Kd | 现象 | 结论 |
|----|------|------|
| 0 | 响应柔，过冲稍大 | 可用但不够锐 |
| 0.05 | 爬升时光流帧间跳变被 D 放大→倾角振荡 | 太大 |
| 0.02 | ✅ 轻阻尼，不过冲 | 最佳 |

#### 角度限幅的奇妙行为

| 限幅 | 现象 |
|------|------|
| 5° | 高频振荡，频率约为 3~4Hz |
| 8° | 中频振荡 2Hz |
| 10° | 低频振荡 <1Hz |
| 12° | ✅ 稳定，晃动缓慢可收敛 |
| 14° | 响应足够，但噪声晃动幅度略大 |

反常现象：**更大的限幅 → 更低频率的振荡**。
原因：限幅大 → 系统出力更足 → 快速回正 → 但 overshoot 后修正力也大 → 周期变长。
限幅小 → 每次修正量有限 → 回正慢 → 累积误差后持续小幅度修正 → 高频抖。

#### X/Y 轴分离整定

X 轴（前后/Pitch）：扰动小，起飞方向与机身对称轴一致 → P 和 I 都略小。
Y 轴（左右/Roll）：机械装配不对称（电池/线缆偏一侧），噪音略大 → P 和 I 略大。

### YAW 锁定优化

| 调整 | 前值 | 后值 | 效果 |
|------|------|------|------|
| pid_yaw_angle.kp | 1.12 | 1.8 | 加强外环锁定 |
| pid_yaw_angle.out_limit | 80 | 55 | 减少外环过冲 |
| pid_yaw_rate.kp | 7.0 | 10.0 | 加强角速度响应 |
| pid_yaw_rate.ki | 0.20 | 0.10 | 减半防 windup |
| pid_yaw_rate.i_limit | 400 | 200 | 配合 Ki 减半 |

**关键发现**：
- 悬停段偏航非常稳（YR≈0，YL 极小），证明前馈表良好
- 爬升段偏航振荡是横滚/俯仰振荡耦合的**结果**而非原因
- 光流速度环切入时 corr_engage 渐变 500ms 防止偏航冲击
- `YAW_FF_ENABLE=0`（当前关闭前馈）可诊断前馈是否必要

### 爬升段稳定性演进

| 版本 | 问题 | 解决 | 效果 |
|------|------|------|------|
| 1 | 旋转补偿滞后 | 3→8Hz + last_gyro | 相位滞后 53→20ms |
| 2 | IMU 融合污染 | 关闭 IMU 融合 | 纯光流更稳定 |
| 3 | FA 反复切换 | Vz 门限 40→50 + 切入平滑 500ms | 爬升关速度环，到顶平滑恢复 |
| 4 | 积分 windup | |Vz|>12 冻结积分 | 恢复时修正不反转 |
| 5 | D 项过大 | VY_KD 0.05→0.02 | 爬升不振荡 |
| 6 | 速度跳变炸机 | UPF_MAX_VEL_CMS 250→80 | 蓝布低纹理假速度不灌进控制 |

### LC-302 vs PMW3901 对比

| 特性 | LC-302 (当前) | PMW3901 (早期使用) |
|------|---------------|-------------------|
| 接口 | UART 异步 19200bps | SPI 同步 |
| 数据率 | 20~50Hz 不定 | 固定 100Hz |
| 输出 | 像素积分 + 积分时间 | 像素位移 (Δx, Δy) |
| 帧同步 | XOR 校验 + finsh_flag | 每次读都是新数据 |
| 物理速度计算 | v = (pixel/10000)/dt × h × scale | 需要外部位移积分 |
| 有效帧判定 | timespan 门控 + valid 位 + 超时 | 总有帧，质量靠 SQUAL |
| 纹理适应性 | 蓝布/光滑面会出假速度 (130~170cm/s) | 类似问题 |
| 安装方向 | 模块 X→飞机 Y，需互换 | 原生对齐 |

**关键差异**：
- LC-302 异步 UART 意味着可能 20ms 内有 0、1 或 2 帧，必须有超时/有效帧管理
- PMW3901 同步 SPI 每次调用一定有新数据，逻辑更简单但占用 SPI 总线
- LC-302 自带积分时间，高度换算更直接；PMW3901 需要外部时间基准

---

## 串口调试变量手册

每行 20Hz 打印，包含全部控制链路变量，用于 Excel 数据分析。

```
格式:
UPF FA:%d valid:%d fresh:%d H:%.1f Vz:%.1f
  rx:%.1f ry:%.1f tvx:%.1f tvy:%.1f vx:%.2f vy:%.2f
  fx:%.1f fy:%.1f imx:%.1f imy:%.1f
  fw:%.2f pc:%.2f rc:%.2f evx:%.1f evy:%.1f
  op:%.2f or:%.2f ivx:%.1f ivy:%.1f
  ROL:%.1f PIT:%.1f TGTY:%.1f YR:%.1f YRC:%.1f YL:%.0f YI:%.1f
  cv:%d u:%d v:%d ar:%d rx:%u
  PO:%.0f RO:%.0f YO:%.1f PR:%.1f RR:%.1f
```

| 变量 | 含义 | 正常范围 | 异常诊断 |
|------|------|---------|----------|
| FA | flow_active | 0/1 | 应随 Vz 切换 |
| valid | 光流有效位 | 0/1 | 高度低/丢帧时=0 |
| fresh | 最近 150ms 有数据 | 0/1 | 连续=0→通讯中断 |
| H | 当前高度 cm | 0~300 | 应与实际一致 |
| Vz | 垂直速度 cm/s | ±30 | >50→关速度环 |
| rx/ry | 相对位置 cm (poshold 或 cam) | ±60 | 随漂移缓慢变化 |
| tvx/tvy | 目标速度 cm/s | ±18 | 0=悬停，非0=跟车 |
| vx/vy | 光流实测速度 cm/s | ±20 | 有旋转补偿后应接近真值 |
| fx/fy | 融合速度 cm/s | =vx (融合关闭) | α=0 时等于 vx |
| imx/imy | IMU 积分速度 cm/s | ±20 | >50→融合已崩溃 |
| fw | flow_weight | 0~1 | 10~15cm 渐入 |
| pc/rc | pitch/roll 修正角 deg | ±12 | 应随漂移方向修正 |
| evx/evy | 速度误差 cm/s | ±20 | target=0 时 = -v |
| op/or | PI 原始输出 deg | ±12 | 限幅前 |
| ivx/ivy | PI 积分值 | ±8 | 持续增长=恒力未补偿 |
| ROL/PIT | 实际姿态角 deg | ±5 (悬停) | >10→失控趋势 |
| TGTY | 锁定目标航向 deg | 0~360 | 仅起飞前更新 |
| YR | 实测偏航角速度 deg/s | ±5 (悬停) | >10→偏航振荡 |
| YRC | 目标偏航角速度 deg/s | ±55 | 外环输出 |
| YL | 混控偏航量 | ±520 | PID + 前馈 |
| YI | 偏航积分 | ±200 | 大→前馈表不准 |
| cv | 摄像头锁定 | 0/1 | 0=丢目标 |
| u/v | 像素偏移 | ± | |
| ar | 亮点数 | 4~6000 | 太大=过曝，太小=丢目标 |
| PO/RO | 内环最终混控值 | ±1200 | 应小于 out_limit |
| PR/RR | 角度环目标角速度 deg/s | ±25 | 大→角度误差大 |

---

## 调试方法论

### 分析流程

```
串口日志 (20Hz) → Excel 导入 → 按时间画折线图
  └─ 姿态: ROL + PIT + pc + rc → 修正方向对不对？
  └─ 速度: vx + vy + tvx + tvy → 跟踪上了吗？
  └─ 效率: op/or 与 ivx/ivy → P 和 I 谁在出力？
  └─ 偏航: YR + YL + YI → 前馈够不够？振荡？
  └─ 高度: H + Vz + FA → 门控是否正确？
```

### 单变量原则

每次只改一个参数，飞一次，收数据，分析，再改下一个。
不要一次改多个参数——无法判断哪个起了作用。

### 调试顺序

1. **姿态 PID**：先飞稳姿态（不开光流），P 从 8 开始，加 I 消静差，再加 D 防过冲
2. **旋转补偿**：地面摇晃测试验证符号，飞起来验证补偿效果
3. **速度环 P**：从 0.15 逐步加，找到稳定边界（响应快但不高频振荡）
4. **速度环 I**：加小 I 抗恒力漂移，配合泄漏积分防止 windup
5. **速度环 D**：轻阻尼（0.02）减少过冲，不能大（0.05 爬升就振荡）
6. **角度限幅**：从 5° 开始加，找到稳定边界（12° 最佳）
7. **位置环**：先 mock 后真相机，先速度跟踪后位置跟随
8. **YAW 锁定**：最后调，等水平轴稳定后再优化

---

## 代码规范与设计哲学

### 注释规范

每个文件头部应有职责声明和调用时序。关键参数应备注调试历程（初始值→中间值→最终值+原因）。
宏定义旁应有`/* 旧值→新值，原因 */`的演进记录。

### 设计原则

1. **分层解耦**：数据源(A) ⇢ 外环(B) ⇢ 内环(C)，接真相机时只换 A
2. **单点修改**：所有宏集中在一个头文件 (UP_FLOW_302.h)，不改 .c
3. **调试变量**：所有关键中间变量都有 dbg_ 前缀的全局 extern，无需重新编译即可打印
4. **安全冗余**：积分限幅、输出限幅、角度限幅、动力余量保护、坠机检测多重保护
5. **状态可见**：起飞/飞行/落地状态全部通过串口打印，飞控行为完全透明
6. **符号统一**：所有极性宏（_SIGN）都在头文件集中管理，改符号不涉及算法逻辑

### 遗留问题

- 蓝布/低纹理地面光流会出假速度 (130~170cm/s)，靠跳变剔除 (>80) 防护，不是根本解决
- 光流速度 LPF α=0.15 滞后约 3 帧，快速机动（急刹/急转）会跟踪延迟
- Yaw 前馈表只有 7 个断点，中间段靠线性插值，精度有限
- CM7_1 视觉 IPC 不含 ECC 校验，SRAM 软错误理论上有风险（实际已关 ECC）
