# 阶段 1 — 键盘输入器 · 变更与测试证据

日期: 2026-09-24 · 工作区: WSL `/home/dev/dune/dune`
状态: **暂停，等待主控复核**（通过后进入阶段 2）

## 变更清单

| 路径 | 说明 |
|------|------|
| `programs/teleop/keyboard_input.py` | 独立 Python 键盘输入器（**仅标准库**） |
| `programs/teleop/README.md` | 最小运行说明与字段契约 |
| `programs/teleop/test_phase1.py` | 可重复验收脚本（无第三方依赖） |
| `doc/TouchDIVER_Phase1_Keyboard.md` | 本证据记录 |
| `doc/compose/spec/touchdiver-computer-prototype.md` | T1 勾选 |

**未改动**：IMC schema、DUNE C++ 任务、用户 `*.bak*` 备份、Handoff。

## 实现要点（对照任务书阶段 1）

- 终端**非阻塞**按键（`termios` + `select` + `os.read`）
- 按键：`a/z` Grip0，`s/x` Grip1，`0` 归零，`e` 使能/停用，`q`/Ctrl+C 安全退出
- 轴值归一化 **-127..127**，步长默认 10
- **10 Hz** 周期发送 IMC v5 **RemoteActions**(305)；循环内**不**调用 `dune-sendmsg`
- 启动时向 Discovery 默认五端口发送 IMC **Announce**(151)，每 10 秒刷新一次；源地址 `0xFFEF` 与 `dune-sendmsg` 一致，供 UDP 动态 contact 注册
- 直接编码 IMC：sync `0xFE54` + 头 20 字节 + `u16 len + utf-8` payload + **CRC16-IBM** 尾 2 字节
  （对齐 `IMC::Packet::serializeHeader` / `Algorithms::CRC16` / `serialize(std::string)`）
- TupleList 文本（默认 `=` / `;`，与 `DUNE::Utils::TupleList` 一致）:

```text
Enable=<0|1>;Stop=<0|1>;Grip0=<-127..127>;Grip1=<-127..127>;
```

- **按下 `e` 使能前不发送非零**（未使能时 Grip 强制 0）
- 退出前尽力发送 **零值 + Stop=1**，并恢复终端 (`tcsetattr`)
- CLI：`--host/--port/--discovery-port/--announce-period/--rate/--step`；业务端口默认 `127.0.0.1:6002`，Discovery 起始端口默认 `30100`
- 状态行显示目标、使能、两轴、发送计数

## 测试命令与结果

### 1) 编解码自检

```bash
python3 programs/teleop/keyboard_input.py --selftest
```

**PASS** — 输出 `selftest OK`

样本 hex（timestamp=1000）: `54fe310125000000000000408f40efffffffffff2300456e61626c653d313b...`

`actions: Enable=1;Stop=0;Grip0=50;Grip1=-30;`
覆盖：CRC、RemoteActions id=305、Announce id=151、未使能归零、±127 夹紧。

### 2) 行为与 10 Hz 验收

```bash
python3 programs/teleop/test_phase1.py
```

**PASS** — 全部断言通过：

```text
test_keys OK
test_disabled_no_nonzero OK
test_announce OK
test_stop_payload OK
test_rate_and_udp packets_sent=25 received=25 elapsed=2.406 hz≈9.97
avg_gap=0.1002 s
test_rate_and_udp OK
test_tty_exit q_and_sigint OK
ALL_PHASE1_CHECKS_OK
```

| 验收项 | 结果 |
|--------|------|
| 无额外第三方包 | PASS（stdlib only） |
| 按键行为可复现 | PASS（test_keys） |
| 连续约 10 Hz | PASS（9.97 Hz，avg gap 0.1002 s） |
| 未使能轴保持 0 | PASS |
| Stop/退出载荷 | PASS（test_stop_payload） |
| `q` / SIGINT 退出、Stop=1 和终端恢复 | PASS（伪终端运行真实输入器进程；属性与启动前一致） |

### 3) 与 DUNE 的协议互操作

- 使用 `/home/dev/dune/build/dune-sendmsg` 向临时 UDP 端口发送 Heartbeat，同时抓取其 Announce。Python 解码到 `msg_id=151`、`src=65519`、`sys_name=dune-sendmsg`、`services=no;services`。**PASS**，与本地 DUNE 工具的 Announce 格式一致。
- 用 `/home/dev/dune/build/dune` 和临时配置副本启动隔离仿真实例：UDP `16002`、Discovery `31100`，使用独立组播地址且无静态 UDP 目的地址。Python 发送五条 Announce 和四条约 10 Hz 的 RemoteActions。
- DUNE 日志记录 `new node within range 'touchdiver-keyboard' / 65519 / 127.0.0.1`，并在 UDP Listener 记录 `incoming: RemoteActions` 四次。**PASS**。
- 隔离实例已停止。原有 PID `41105` 未停止或修改。隔离日志保留于 `/tmp/touchdiver-phase1-isolated.OgPE6V.log`，临时配置副本位于 `/tmp/touchdiver-phase1-etc.Zx254R`。

## 未解决问题 / 说明

- 阶段 1 验证键盘输入、IMC 编码和 DUNE Discovery/UDP 收包；DUNE 目前还没有解析 Grip 字段并驱动虚拟舵机的阶段 2 Task，因此未验证舵机动作。
- 字段名 `Grip0/Grip1/Enable/Stop` 为阶段 2 DUNE Task 解析契约；本阶段不写 C++。
- RemoteActions 仍为**实现选择**（见阶段 0 更正说明），非限速推导的必要条件。

## git

```text
## master...origin/master
?? programs/teleop/   (含本阶段新文件)
?? doc/... 等原有未跟踪
?? etc/... bak 与 src/...Task.cpp.bak 未改动
```

`git diff --check`：工作树无空白错误（新文件为未跟踪，检查方式同阶段 0）。
