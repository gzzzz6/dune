---
feature: touchdiver-computer-prototype
status: delivered
updated: 2026-09-24
branch: master
commits: n/a
---

# TouchDIVER 电脑端原型（键盘 → DUNE 虚拟舵机）

> **权威需求**：doc/TouchDIVER_Computer_Prototype_Handoff.md。与任务书冲突时以任务书为准。
> 工作区：WSL /home/dev/dune/dune。续跑时若无 Compose Next 指令，先重新加载 compose-next skill。

## Report

**What was built** — 键盘 IMC RemoteActions 输入器、Control.Manipulator.Teleoperation（虚拟舵机）、Control.Manipulator.Haptics（合成/舵机接触→DevDataText 抽象触觉命令），Simulation 专用配置与阶段 0–3 及最终交接文档。

**Verification** — make dune PASS；键盘→舵机映射 0.26386/0.20614/0.10307 rad 与公式一致；合成 level 扫描至 99（范围 0..100，历史见 100）；舵机 contact level=52/76；NaN→invalid-feedback；静默→feedback-timeout；启停/并发/超时归零均实测；git diff --check 干净；.bak 未改。

**Journey log** —
1. DUNE_TASK 宏不可省，否则链接失败。
2. Parameter::values() 不能用于 int，任务无法加载。
3. onMain 忙循环刷指令会饿死消息泵；输出须 10 Hz 限速。
4. 外部 UDP 需先 Announce；过早发包只见 incoming 不进 consume。
5. TextMessage 会被 Plan.Generator 误解析，抽象命令用 DevDataText。

## [S1] Problem

无机械臂/潜航器/实体手套时，打通「输入 → IMC → DUNE 控制 → 虚拟舵机」链路，并为 WEART SDK 预留输入边界。

## [S2] Design

1. Python 键盘程序 10 Hz 发 IMC v5 RemoteActions（Grip0/Grip1、Enable/Stop，轴值约 -127..127）。RemoteActions 为实现选择，非由 UDP 出站限速推导的必要条件。
2. 新增 DUNE Task：解析 RemoteActions → SetServoPosition 驱动 Simulators.Servos（0/1 通道，默认 ±30° 夹紧 ±45°）；500 ms 超时归零；仅 Simulation。
3. KeyboardInput 抽象边界，未来 WeartInput 同接口。不改 IMC、无 IK、无真实硬件。
4. 阶段 3：合成接触 → 抽象触觉命令。

已核验：IMC 5.4.31；SetServoPosition=302 (id,value)；ServoPosition=281；RemoteActions=305 (actions:string)；UDP 6002；Rate Limiters SetServoPosition:1 作用于 UDP 出站 consume 转发（非入站）；Servos Maximum Angle 45。

## [S3] Out of Scope

真实 SDK、IK、改 IMC XML、真实硬件、清理用户 .bak 文件。

## Tasks

- [x] T0: 阶段 0 基线与协议验证 (covers: S2) — 见 doc/TouchDIVER_Phase0_Baseline.md
- [x] T1: 阶段 1 Python 键盘输入器 (covers: S2; depends: T0) — 见 doc/TouchDIVER_Phase1_Keyboard.md
- [x] T2: 阶段 2 DUNE Teleoperation Task (covers: S2; depends: T1) — 见 doc/TouchDIVER_Phase2_DUNE_Task.md
- [x] T3: 阶段 3 合成触觉 + 故障 (covers: S2; depends: T2) — 见 doc/TouchDIVER_Phase3_Haptics.md — 实现中
