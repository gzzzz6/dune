# 阶段 3 — 合成接触与抽象触觉命令 · 变更与测试证据（复核补验版）

日期: 2026-09-24 · 工作区: WSL `/home/dev/dune/dune`  
状态: **部分通过，待复核**（实现完成度通过；本文件已补运行时故障测试与文档一致性修正）  
**不进入最终交接验收**，直至主控确认。

## 相对上轮的修正

| 复核意见 | 处理 |
|----------|------|
| 需重新提供可执行证据 | 本轮在 `/home/dev/dune/build/dune`（18,213,168 B，2026-09-24 17:07）上重跑 |
| invalid-feedback / feedback-timeout 仅代码路径 | **已运行时实测**（见 D） |
| 注释仍写 TextMessage | **已改为 DevDataText** |
| T3 误标 `[x]` | **已改回 `[ ]`（待复核）** |
| 去重未比较 axis/level | **已扩展**为 axis+level+active+reason 一致才 100 ms 合并；强度/轴变化不丢弃（B 中 level 2→52 阶梯可见） |

## 变更清单

| 路径 | 说明 |
|------|------|
| `src/Control/Manipulator/Haptics/Task.cpp` | 合成接触 + 舵机虚拟接触 → `DevDataText` |
| `src/Control/Manipulator/Haptics/Task.cmake` | 空文件 |
| `etc/auv/control.ini` | `[Control.Manipulator.Haptics]` Enabled=Simulation |
| `doc/TouchDIVER_Phase3_Haptics.md` | 本证据 |
| `doc/compose/spec/touchdiver-computer-prototype.md` | T3 待复核 |

## 抽象触觉命令格式

`IMC::DevDataText`（id **273**）`value` 字段：

```text
HAPTIC|axis=<0|1>|level=<0..100>|active=<0|1>|reason=<...>
```

reason：`startup` | `shutdown` | `synthetic-contact` | `contact` | `clear` | `feedback-timeout` | `invalid-feedback`

---

## 本轮运行时证据（2026-09-24 重跑）

### 构建与启动

| 命令 | 结果 |
|------|------|
| `cmake && make dune` | **PASS** MAKE:0 |
| `ls build/dune` | **PASS** `-rwxr-xr-x … 18213168 Sep 24 17:07 /home/dev/dune/build/dune` |
| `./dune -d /tmp/dune-etc -c lauv-simulator-1 -p Simulation` | **PASS** 起停均返回，日志见 `/tmp/r3a.log` `/tmp/r3b.log` `/tmp/r3e.log` |

### A. 合成接触可重复闭环 — **PASS**

| 项 | 值 |
|----|-----|
| `reason=startup` | 1 |
| `reason=synthetic-contact` | **67** |
| `reason=shutdown` | 1 |
| 强度样例 | level **1,6,12,17,…** 最高 **99**（三角波） |
| 样例行 | `HAPTIC\|axis=0\|level=12\|active=1\|reason=synthetic-contact` |

### B. 舵机链路接触映射 — **PASS**

`Grip0=100` → Teleoperation → Servos → Haptics：

| 项 | 值 |
|----|-----|
| contact 爬升 | level **2→9→17→24→32→34→41→48→52** |
| level=52 条数 | **16** |
| 公式 | 100/127×30° ÷ 45° × 100 ≈ **52** |
| 去重修复验证 | 同 reason 下 level 变化**未被吞掉** |

### C. 启动 / 重启 — **PASS**

`startup` / `shutdown` 各 1 次/次启停，退出无残留 `active=1`。

### D. 运行时故障注入 — **PASS**（本轮新增）

隔离条件：`Synthetic Contact=false`，`Simulators.Servos=Never`，`Teleoperation=Never`；先 `Announce` 注册源，再发 `ServoPosition`(281)。

| 步骤 | 注入 | 日志 |
|------|------|------|
| D1 | `value=NaN` | `incoming: ServoPosition` + **`reason=invalid-feedback`** |
| D2 | `value=0.6` rad 后静默 | **`reason=contact` level=76**（0.6/0.785×100） |
| D3 | 0.5 s 无更新 | **`reason=feedback-timeout`** |

markers 计数：`startup×1, invalid-feedback×1, contact×1, feedback-timeout×1, shutdown×1`

### E. 双输入端并发 — **PASS（实测+说明）**

两线程同时发 RemoteActions；后到覆盖轴状态；无崩溃（阶段 3 前轮 + 阶段 2）。

---

## 故障场景（任务书）

| 场景 | 行为 | 状态 |
|------|------|------|
| 清除接触 | `clear, active=0` | **已实测** |
| 遥操作输入超时 | 舵机归零 → Haptics clear | **已实测** |
| 反馈超时 | `feedback-timeout` | **已实测（D3）** |
| 无效反馈 | `invalid-feedback` | **已实测（D1）** |
| 退出/重启 | `shutdown`/`startup` | **已实测** |
| 双端并发 | 后到覆盖 | **已实测** |
| 非法 RemoteActions | 不刷新看门狗（阶段 2） | **已实测** |

## 去重策略（修正后）

100 ms 内仅当 **axis、level、active、reason 全部相同** 才合并；强度或轴变化立即输出。属有意的限频采样，不丢有效变化。

## 已验证 vs 待实物

| 软件链路已验证 | 本原型不宣称 |
|----------------|--------------|
| 合成接触→抽象命令可重复 | 真实触感/力反馈 |
| 舵机→强度映射（level=52/76） | 2 mm 追踪精度 |
| 无效/超时/启停/并发 | 机械臂运动学、真实时延、SDK 效果 |

## 构建注意

- `Parameter::values()` 不可用于 int 参数（会阻止加载）→ 使用 `bool Synthetic Contact`
- 输出消息为 **DevDataText**（非 TextMessage），避免 Plan.Generator 误解析
- 外部注入 ServoPosition 前需 **Announce** 注册

## git

```text
M  etc/auv/control.ini
M  src/Control/AUV/Attitude/Task.cpp   (仅中文注释)
?? src/Control/Manipulator/{Teleoperation,Haptics}/
?? doc/TouchDIVER_Phase{0,1,2,3}_*.md
?? programs/teleop/
?? *.bak 未改动
```

`git diff --check`：干净。
