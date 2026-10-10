# 阶段 2 — DUNE Teleoperation Task · 变更与测试证据

日期: 2026-09-24 · 工作区: WSL `/home/dev/dune/dune`
状态: **阶段 2 完成，暂停等待复核**（通过后进入阶段 3）

## 变更清单

| 路径 | 说明 |
|------|------|
| `src/Control/Manipulator/Teleoperation/Task.cpp` | 新增任务（C++98 风格，`DUNE_TASK` 导出） |
| `src/Control/Manipulator/Teleoperation/Task.cmake` | 空文件（任务自动发现） |
| `etc/auv/control.ini` | `[Control.Manipulator.Teleoperation]`，**Enabled = Simulation** |
| `doc/TouchDIVER_Phase2_DUNE_Task.md` | 本证据 |

**未改动**：IMC schema、AUV Attitude/Allocator/RemoteOperation、用户 `*.bak*`、Handoff。

## 实现要点

- 模块路径：`Control.Manipulator.Teleoperation`（独立任务，不混入 AUV 姿态/推进远控）
- 消费 `IMC::RemoteActions`(305)，`TupleList` 字段：`Enable` `Stop` `Grip0` `Grip1`
- 映射：`axis/127 * Maximum Command Angle(默认 30°)` → `Angles::radians` → `SetServoPosition.value`（弧度）
- 通道：Grip0→Servo Id 0，Grip1→Servo Id 1（可配置）
- 非法轴值（\|raw\|>127）视为 0；**Enable=0 或 Stop≠0 → 轴强制 0**
- `Enable`、`Stop`、`Grip0`、`Grip1` 必须全部存在且为严格整数；缺失、非数字或越界报文会被忽略，**不会刷新超时计时器**
- `Servo Id 0/1` 在运行时限制到 `0..255`，避免配置值转换为 `uint8_t` 时溢出
- **Input Timeout 0.5 s**：无有效输入则零输出并撤销使能（war 一次直到新输入）
- `onMain` 以 **10 Hz** 限速 dispatch（避免刷爆总线）；`waitForMessages` 正常泵入 RemoteActions
- 默认 **不驱动真实硬件**；不改 AUV 导航/姿态行为

## 测试命令与结果

```bash
cd /home/dev/dune/build
cmake --build . --target dune -j2
# 启动仿真：
./dune -d /tmp/dune-etc -c lauv-simulator-1 -p Simulation
# 等待约 6 s 后：
./dune-sendmsg 127.0.0.1:30100 6002 RemoteActions 'Enable=1;Stop=0;Grip0=64;Grip1=0;'
./dune-sendmsg 127.0.0.1:30100 6002 RemoteActions 'Enable=1;Stop=0;Grip0=0;Grip1=-64;'
./dune-sendmsg 127.0.0.1:30100 6002 RemoteActions 'Enable=0;Stop=0;Grip0=90;Grip1=90;'
# keyboard_input.send_state → Grip0=50 Grip1=25
```

### 结果（PASS）

| 验收项 | 证据 |
|--------|------|
| DUNE 编译 | `cmake --build . --target dune -j2` **PASS**；`Control.Manipulator.Teleoperation` 编译并链接 |
| 启动仿真后键盘/输入器改两路舵机 | Grip0=64 → servo0 **0.26386 rad (15.12°)**；Grip1=-64 → servo1 **-0.26386 rad** |
| 双轴独立 | Grip0=50/Grip1=25 → **0.20614 / 0.10307 rad** |
| 角度不超限 | 最大 15.12° < 30° 软限幅；模拟器夹紧 ±45° |
| 停用无非零 | Enable=0 且 Grip=90,90 → 输出保持 **0** |
| 超时归零 | 0.5 s 无输入 → `input timeout - outputs zeroed`，轴清零 |
| 非法输入不喂狗 | 缺字段、非数字、`Grip0=128` 均记录 `invalid RemoteActions input - ignored`；随后仍触发 0.5 s 超时 |
| 舵机 ID 配置边界 | 参数更新时夹紧到 `0..255`，避免窄整数转换溢出 |
| LSF SetServoPosition | s0 集合 **{0, 0.20614, 0.26386}**；s1 **{-0.26386, 0, 0.10307}**，与期望一致 |
| 键盘输入器路径 | `keyboard_input.send_state` 同样驱动两路（RemoteActions: 日志可见） |
| 不影响 AUV 导航/姿态 | 无 Attitude/Allocator 错误；仅 GSM/Iridium COMM 警告为 **PRE-EXISTING** |

期望值验算：`64/127*30° = 15.12° = 0.26386 rad` 等，与 LSF 完全一致。

## 构建注意

- 新增 Task 由 `cmake/Tasks.cmake` 递归发现 `Task.cmake`；`Control.Manipulator.Teleoperation` 已成功编译并链接
- `DUNE_TASK` 导出宏不可省略，否则链接期 `createControlManipulatorTeleoperationTask` 未定义
- 配置参数须与 Task 声明一致（多余键会报 `invalid parameter`）

## 测试时序注意

DUNE 完全就绪约需 **≥6 s**；过早发送的 RemoteActions 可能只出现在 UDP `incoming` 而尚未绑定到消费者。验收脚本在 `sleep 6` 后发送。

## 未做 / 留给阶段 3

- 合成接触 → 抽象触觉命令
- 双输入端并发的系统化边界测试
- 真实手套 / 机械臂

## git

```text
M  src/Control/AUV/Attitude/Task.cpp        (仅中文注释，非本阶段逻辑改动)
?? src/Control/Manipulator/Teleoperation/   (新任务)
M  etc/auv/control.ini                      (仅新增 Simulation 段)
?? doc/... 阶段 0/1 证据与 programs/teleop
?? *.bak 备份未改动
```
