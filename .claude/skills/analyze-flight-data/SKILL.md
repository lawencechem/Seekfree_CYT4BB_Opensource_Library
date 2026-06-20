---
name: analyze-flight-data
description: Autonomously parse flight data (test.txt), identify all control loop issues, pinpoint divergence root cause, and suggest code fixes
metadata:
  type: project
  unit: Seekfree CYT4BB 开源库
---

# Analyze Flight Data — 飞控数据自动分析

自动读取 `test.txt` 飞行日志，逐项分析各控制环路，找出系统发散根因，给出多个可实施的代码修改方案。

## Usage

```bash
python .claude/skills/analyze-flight-data/driver.py [path/to/test.txt]
```

默认读取项目根目录的 `test.txt`。

## What it analyzes

| # | 项目 | 判断依据 |
|---|------|---------|
| 1 | 光流数据 | fresh, valid, vx/vy 连续性，低通滤波效果 |
| 2 | 摄像头数据 | cv, cx/cy, u/v, ar, mg — 是否锁到目标 |
| 3 | 速度环 PID | pc/rc, evx/evy, op/or, vx/vy 跟踪能力，是否饱和 |
| 4 | 位置环 PID | tvx/tvy vs cx/cy, 是否介入，输出是否饱和 |
| 5 | 姿态环 PID | PO/RO, PR/RR, PD/RD, ROL/PIT 响应 |
| 6 | Yaw PID | YRC, YR, YI, YL — 积分是否饱和，故障检测是否触发 |
| 7 | 角度限幅 | PR/RR 是否被 ±60 限幅截断 |
| 8 | 爬升速度 | Vz, CLIMB_UP_MAX_SPEED 限幅是否生效 |

## Output

- 每个子系统的诊断结论（OK / WARNING / FAIL）
- 系统发散根因判断
- 多个可选的代码修改方案（用户确认后才执行）

## Notes

- 数据最后 1~2 秒为用户手提飞机数据，分析时会扣除
- 所有修改需用户确认，不自动写代码
