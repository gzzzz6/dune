# Phase 1 — 键盘输入器运行说明

独立 Python 程序：`keyboard_input.py`
路径：`programs/teleop/keyboard_input.py`
依赖：**仅 Python 3 标准库**（socket / termios / struct / select…），无需 pip。

## 自检（无网络、可重复）

```bash
python3 programs/teleop/keyboard_input.py --selftest
```

预期输出：`selftest OK` 及一行 RemoteActions hex。校验内容：

- IMC v5 头 20 字节 + payload + CRC16-IBM
- `RemoteActions` id=305，payload 为 `u16 len + utf-8`
- 启动时向 DUNE Discovery 的五个端口发送 `Announce`(151)，并每 10 秒刷新一次来源注册
- TupleList：`Enable=..;Stop=..;Grip0=..;Grip1=..;`（与 DUNE `TupleList` 默认 `=` / `;` 一致）
- 未使能时轴值强制 0；轴值夹紧 ±127

## 交互运行

1. 启动 DUNE 仿真（阶段 0 命令）。
2. 在**真实终端**中运行：

```bash
python3 programs/teleop/keyboard_input.py --host 127.0.0.1 --port 6002 --rate 10
```

如果 DUNE UDP 端口被另一个实例占用，DUNE 会尝试后续端口；运行参数应与日志中的实际监听端口一致。Discovery 默认从 `30100` 起使用五个端口，可用 `--discovery-port` 指定起始端口。

3. 按键：

| 键 | 作用 |
|----|------|
| `a` / `z` | Grip0 增 / 减（步长默认 10） |
| `s` / `x` | Grip1 增 / 减 |
| `0` | 两轴归零 |
| `e` | 使能开关（关闭时轴强制 0，且不发送非零） |
| `q` 或 `Ctrl+C` | 安全退出：发送零值 + `Stop=1` 后恢复终端 |

状态行显示：目标地址、使能、两轴当前值、发送计数。

## 行为约定（验收对照）

- 周期发送 **约 10 Hz**（`--rate` 可调），循环内**不**调用 `dune-sendmsg`
- 启动时发送 IMC `Announce`，此后每 10 秒刷新一次，用于通过 DUNE UDP 动态 contact 过滤
- **按下 `e` 使能之前不发送非零输入**（轴保持 0）
- 退出 / Ctrl+C：尽力发送零值与 `Stop=1`，并 `tcsetattr` 恢复终端
- CLI 可指定 IP/端口，默认 `127.0.0.1:6002`

## 与阶段 2 的字段契约

`RemoteActions.actions`（TupleList）：

```text
Enable=<0|1>;Stop=<0|1>;Grip0=<-127..127>;Grip1=<-127..127>;
```

DUNE 侧应使用 `Utils::TupleList(msg->actions)` 默认分隔符解析同名字段。
`RemoteActions` 为周期输入协议的**实现选择**；字段名固定，后续 `WeartInput` 映射到同一轴值即可。

## 阶段 1 验收

```bash
python3 programs/teleop/test_phase1.py
```

脚本包含本机 UDP 频率检查，以及通过伪终端验证 `q` / SIGINT（终端 Ctrl+C 对应的信号）退出、停止报文和终端属性恢复。
