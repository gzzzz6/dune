# TouchDIVER 电脑端原型 — 最终交接验收

日期: 2026-09-24 · 工作区: WSL `/home/dev/dune/dune`  
对应任务书: `doc/TouchDIVER_Computer_Prototype_Handoff.md`  
阶段: 0–3 全部完成并通过复核

---

## 1. 端到端演示步骤（可复现）

### 1.1 键盘 → 虚拟舵机（软件链路 1）

```bash
# 终端 A：仿真（勿用 Neptus 也可）
cd /home/dev/dune/build
./dune -d /home/dev/dune/dune/etc -c lauv-simulator-1 -p Simulation
# 等待约 6 s 至任务全部 starting

# 终端 B：键盘输入器（需真实 TTY）
python3 /home/dev/dune/dune/programs/teleop/keyboard_input.py --host 127.0.0.1 --port 6002 --rate 10
# 按 e 使能，a/z 调 Grip0，s/x 调 Grip1，0 归零，q 安全退出
```

**预期**：约 10 Hz 发送 `RemoteActions`；虚拟舵机角度随按键变化（可用 Debug Level=Spew 看 `axis=` 日志）；停止输入 0.5 s 后归零。

### 1.2 合成接触 → 抽象触觉命令（软件链路 2）

```bash
# control.ini: [Control.Manipulator.Haptics] Synthetic Contact = true
cd /home/dev/dune/build
./dune -d /home/dev/dune/dune/etc -c lauv-simulator-1 -p Simulation
# Debug Level=Spew 时可见：
# HAPTIC|axis=0|level=1|active=1|reason=synthetic-contact
# HAPTIC|axis=0|level=6|...
```

**预期**：三角波强度扫描；`startup`/`shutdown` 标记。

### 1.3 舵机链路 → 触觉命令（两路闭环）

```bash
# Haptics: Synthetic Contact = false；Teleoperation + Servos 启用
# 用 keyboard_input 设 Grip0=100（或 dune-sendmsg RemoteActions 'Enable=1;Stop=0;Grip0=100;Grip1=0;'）
```

**预期**：`HAPTIC|axis=0|level=52|active=1|reason=contact`（100/127×30°÷45°×100≈52）。

### 1.4 故障注入（可选复测）

```bash
# 关闭 Teleoperation 与 Simulators.Servos 后：
# 1) Announce 注册源（keyboard_input.send_discovery_announces）
# 2) 发 ServoPosition value=NaN  → reason=invalid-feedback
# 3) 发 ServoPosition value=0.6 后静默 0.5s → contact → feedback-timeout
```

---

## 2. 输入适配接口说明（未来 WeartInput）

稳定数据源边界（键盘与未来 WEART 共用）：

```text
InputFrame {
  enabled: bool          // 对应 TupleList Enable
  stop: bool             // 对应 Stop
  axis[2]: float in [-1,1]  // Grip0, Grip1（-127..127 线性缩放）
  timestamp: monotonic   // 有效性/看门狗
  valid: bool
}
```

**映射到 IMC RemoteActions（当前唯一进 DUNE 的输入协议）**：

```text
Enable=<0|1>;Stop=<0|1>;Grip0=<-127..127>;Grip1=<-127..127>;
```

| 源 | 实现 | 文件 |
|----|------|------|
| 当前 | `KeyboardInput` → 10 Hz `RemoteActions` | `programs/teleop/keyboard_input.py` |
| 未来 | `WeartInput`：Middleware/SDK 读追踪与捏合 → 同上字段 | **新建独立程序**，不改 DUNE Task |

**约定**：

1. 不要改 `Control.Manipulator.Teleoperation` 消费的字段名与量纲（±127）。
2. 触觉方向另设输出适配器：真实机械臂力/接触传感器 → 抽象事件 → 映射为 WEART SDK 效果（力/纹/热）；本原型只产 `HAPTIC|...` 抽象命令。
3. 首版不假定 SDK 运行平台、采样率或传感器规格。
4. **接入真机时**：保持 DUNE 姿态/推进/Allocator/安全监控链路；触觉任务只输出抽象命令，**禁止绕过**现有控制与故障保护。

---

## 3. 交付清单（任务书「最终交付」）

| 交付项 | 位置 | 状态 |
|--------|------|------|
| 键盘输入器及运行说明 | `programs/teleop/keyboard_input.py` + `README.md` | ✅ |
| 新增 DUNE 虚拟操纵任务 | `src/Control/Manipulator/Teleoperation/Task.cpp` | ✅ |
| 新增触觉命令任务 | `src/Control/Manipulator/Haptics/Task.cpp` | ✅ |
| Simulation 配置 | `etc/auv/control.ini` 两段 Enabled=Simulation | ✅ |
| 可复现构建与端到端验收记录 | `doc/TouchDIVER_Phase0..3_*.md` | ✅ |
| 输入适配说明 | 本文 §2 | ✅ |

---

## 4. 任务书清单最终核对

| 要求 | 结果 | 证据 |
|------|------|------|
| 键盘模拟输入控制 DUNE 虚拟舵机 | ✅ | Phase 1+2，Grip 映射 0.26386 rad 等 |
| 输入→IMC→控制→模拟执行器链路 | ✅ | RemoteActions→Teleoperation→SetServoPosition→Servos |
| 保留输入适配边界 | ✅ | §2 InputFrame 契约 |
| 首版不接真实手套 SDK | ✅ | 无 SDK 依赖 |
| 不实现机械臂逆运动学 | ✅ | 无 IK 代码 |
| 不修改 IMC 消息定义 | ✅ | Definitions.hpp 未改 |
| 触觉只用合成接触验证命令生成 | ✅ | Haptics 合成源 + 舵机虚拟接触 |
| 不宣称真实触感/力反馈 | ✅ | §5 明确区分 |
| Python 独立程序、非阻塞按键 | ✅ | Phase 1 |
| 禁止循环内 dune-sendmsg | ✅ | 直接 IMC v5 UDP |
| 10 Hz RemoteActions | ✅ | 9.58–9.97 Hz 实测 |
| 使能前不发非零 | ✅ | Phase 1 测试 |
| 退出发零值并恢复终端 | ✅ | Phase 1 |
| DUNE 任务独立模块 | ✅ | Control.Manipulator.* |
| 仅 Simulation 启用 | ✅ | Enabled=Simulation |
| 超时 500 ms 归零 | ✅ | Phase 2 |
| 角度限幅 | ✅ | 默认 30°，模拟器 45° |
| 不影响 AUV 导航/姿态 | ✅ | 无 Attitude 逻辑改动（仅注释） |
| 默认不驱动真实硬件 | ✅ | 仅发 SetServoPosition 至虚拟舵机 |
| C++98 风格 | ✅ | 新任务遵守 |
| 合成触觉命令可重复 | ✅ | Phase 3 A |
| 故障安全输出 | ✅ | invalid-feedback / feedback-timeout / clear / shutdown 实测 |
| 双输入端并发 | ✅ | 后到覆盖 |

---

## 5. 已验证能力 vs 实物待验证

### 已验证（软件链路，可复现）

```text
键盘 / 合成接触
    ↓  RemoteActions (305) / 合成发生器
Control.Manipulator.Teleoperation / Haptics
    ↓  SetServoPosition (302) 或 DevDataText (273)
Simulators.Servos → ServoPosition (281) → Haptics
    ↓
HAPTIC|axis=…|level=…|active=…|reason=…
```

- 两轴独立映射与限幅  
- 合成强度扫描（本轮 max **99**；算法范围 **0..100**，此前记录曾达 **100**）  
- 舵机接触强度（level=52 / 76）  
- invalid-feedback、feedback-timeout、clear、startup/shutdown  
- 双端并发、非法输入不刷新看门狗  

### 实物待验证（本原型不宣称）

| 项 | 说明 |
|----|------|
| 真实触感 / 力反馈效果 | 需 WEART 手套与 SDK |
| 追踪精度 2 mm | 需实物标定 |
| 机械臂运动学与抓取 | 需实机 |
| 真实通信时延 | 需实测 |
| WEART SDK 效果映射 | 未来 WeartInput + 触觉输出适配器 |

### 真机接入原则（交接约束）

保持 DUNE **姿态、推进、执行器分配、安全监控**链路完整；触觉任务只产出**抽象命令**，由输出适配器转 SDK 效果，**不得绕过**现有控制与故障保护。

---

## 6. 阶段证据索引

| 阶段 | 文档 | 结论 |
|------|------|------|
| 0 基线协议 | `doc/TouchDIVER_Phase0_Baseline.md` | 通过 |
| 1 键盘输入器 | `doc/TouchDIVER_Phase1_Keyboard.md` | 通过 |
| 2 DUNE 舵机任务 | `doc/TouchDIVER_Phase2_DUNE_Task.md` | 通过 |
| 3 合成触觉 | `doc/TouchDIVER_Phase3_Haptics.md` | 通过（含故障注入） |
| compose 跟踪 | `doc/compose/spec/touchdiver-computer-prototype.md` | T0–T3 全勾 |

---

## 7. 遗留与建议

1. 合成强度本轮采样 max=99（0.1 s 采样相位）；实现为 0..100 整数，历史轮次曾打印 100。  
2. `Attitude/Task.cpp` 工作区仅有中文学习注释差异，未回退。  
3. 用户 `.bak` 文件全程未改动。  
4. 后续优先：`WeartInput` 对接 WEART Middleware；夹爪力→触觉输出适配器。
