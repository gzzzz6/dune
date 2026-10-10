# 阶段 0 基线与协议验证 — 验收记录

日期: 2026-09-23 · 工作区: WSL `/home/dev/dune/dune` · 构建: `/home/dev/dune/build`

## 环境与版本

| 项 | 值 | 证据 |
|----|-----|------|
| DUNE | 2026.01.0 (master,98d6052,dirty) | 日志 Transports.Logging |
| IMC | 5.4.31, MD5 431862f600b4d042b3eff3664dedc132, sync 0xFE54 | `src/DUNE/IMC/Constants.hpp` |
| Profile | `lauv-simulator-1`, `-p Simulation` | 启动命令见下 |
| UDP | `0.0.0.0:6002` | 日志 listening |
| Discovery | `0.0.0.0:30100` | 日志 listening |
| SetServoPosition | id=302, 字段 `id:u8` + `value:fp32`（弧度） | `src/DUNE/IMC/Definitions.hpp` |
| ServoPosition | id=281, 字段 `id:u8` + `value:fp32` | 同上 |
| RemoteActions | id=305, `actions:string` | 同上 |
| Servos 模拟器 | Maximum Angle=45°, Enable Rotation Rate=true, Maximum Rate=333.3°/s | `etc/auv/simulator.ini` |
| UDP Rate Limiters | `SetServoPosition:1`（1 Hz） | `etc/auv/transports.ini` |

## 仿真启动

```bash
cd /home/dev/dune/build
./dune -c lauv-simulator-1 -p Simulation
# 或带 Neptus: /home/dev/dune/run_sim_with_neptus.sh
```

基线启动无新增错误。已知 PRE-EXISTING 警告:

- Transports.CommManager: Couldn't resolve GSM label / Iridium label

## 单条基线命令

```bash
/home/dev/dune/build/dune-sendmsg 127.0.0.1:30100 6002 SetServoPosition 0 20
```

注意: 参数为 `<id> <角度(度)>`，工具内部 `Angles::radians(val)`。错误写法 `id 0 value 20` 会得到 value=0。

发出报文 JSON 摘要:

```text
abbrev: SetServoPosition, id: 0, value: 0.349065840244
```

(0.3490658 rad = 20°)

运行日志: `Transports.UDP >> incoming: SetServoPosition`

## ServoPosition 反馈（LSF 解析）

日志目录: `build/log/lauv-simulator-1/20260923/153605/Data.lsf.gz`

| 消息 | id | 舵机 | value (rad) |
|------|-----|------|-------------|
| SetServoPosition | 302 | 0 | **0.34906584** (=20°) |
| ServoPosition | 281 | 0 | 0.00243 → 0.0602 → 0.1182 → 0.1770 → 0.2344 → 0.2918 → **0.34906584** 后稳定 |

结论: 模拟器收到指令后按 `Enable Rotation Rate` 逼近目标并持续发布 ServoPosition；四通道均有周期性 ServoPosition。共检出 SetServoPosition×1，ServoPosition 非零 5260 条（含收敛过程）。

## Rate Limiters 方向性说明（更正）

`etc/auv/transports.ini` 中的 `Rate Limiters+ = ... SetServoPosition:1` **不是**对 UDP 入站遥控的限速。

源码依据:

1. `src/DUNE/Tasks/SimpleTransport.cpp` — `param("Rate Limiters", ...)`，在 **`consume(const IMC::Message* msg)`** 里 `if (m_rl.filter(msg)) return;` 后再 `onDataTransmission`（出站）。
2. `src/Transports/UDP/Task.cpp` — **`consume(const IMC::Message* msg)`** 在 `m_filter.filter(msg)` 后 `IMC::Packet::serialize` 并 `m_sock.write`（**DUNE 内部总线 → UDP 对外发送**）。
3. `src/Transports/UDP/Listener.hpp` — 收包路径 `dispatch(msg)`，**未**应用该 Rate Limiters。

因此本阶段证据支持的结论是:

- **从 DUNE 经 UDP 向外转发的 `SetServoPosition` 最多约 1 Hz**。
- **不能**据此认定“UDP 入站 `SetServoPosition` 受 1 Hz 限制”。
- 本阶段观察到 `dune-sendmsg` 入站的 `SetServoPosition` 被 Listener 接收（`incoming: SetServoPosition`），并驱动模拟器产生 `ServoPosition`；入站是否另有业务层限流，阶段 0 **未做双发实验**，不作断言。

对输入协议的影响:

- 任务书选用 **RemoteActions(305) 作为周期键盘输入协议**，属于**实现选择**（便于夹爪轴 + 使能/停止状态打包），**不是**由上述出站 1 Hz 限速推导出的必要条件。
- 无论输入用 RemoteActions 还是直接 SetServoPosition，DUNE 内部 `Simulators.Servos` 的 `consume(SetServoPosition)` 都会即时改目标；开启 `Enable Rotation Rate` 时按 `SimulatedState` 周期逼近。
- 若将来从 DUNE **对外**高频发送 SetServoPosition 给实机，才需考虑 UDP 出站 1 Hz 限制；本原型驱动的是**本机虚拟舵机**（内部总线），不受该出站限速器约束。

## 未做 / 未验证

- Neptus GUI 本轮未启（仅 DUNE 仿真）— 非阶段 0 必需
- UDP 入站 SetServoPosition 双发/1 Hz 入站限速实验 — 非阶段 0 必需
- 真实手套 / 机械臂

## 保留文件

未修改: `*.bak*` 未跟踪备份、Handoff 文档本身。仅新增/修订本验收记录与 compose spec。
