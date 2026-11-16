# NUC 雷达 ↔ 底盘通信协议

> 适用对象：RoboVolley 工程（sentry26-robovolley26），底盘 STM32F407 C 板
> 协议来源：sentry26-main 哨兵工程 NUC 协议，底盘侧实现见 `Module/NUC/nuc.c`
> 默认使用 CAN2，保留 USART6 v1.0 串口协议。修改 `APP/robot_config.h` 中的 `NUC_USE_CAN` 后重新编译，`1` 为 CAN2，`0` 为 USART6。

## CAN2（默认）

NUC 向底盘发送经典 CAN **标准数据帧（11 位 ID），1 Mbit/s**。多字节字段均为小端，CAN ID 直接对应命令号。按表中 DLC 发送原始负载，无需附加串口帧头、序号或 CRC 字段。

| CAN ID | DLC | 负载 | 排球车行为 |
|---|---:|---|---|
| `0x401` | 4 | 字节 0–3：`self_decision`（`uint32`） | 保存自决策值 |
| `0x402` | 5 | 字节 0–1：`vx`（`int16`），字节 2–3：`vy`（`int16`），单位均为 mm/s。字节 4：`flags` | 速度除以 1000 转成 m/s，按下文第 5、6 节执行 |
| `0x403` | 2 | 字节 0–1：`gimbal_yaw`（`int16`），单位 0.01° | 除以 100 保存角度，排球车无云台动作 |

`flags` 的 bit0 为小陀螺、bit1 为云台扫描、bit2 为超电。当前排球车只执行 bit0。例：`vx=+0.5 m/s`、`vy=-0.25 m/s`、开启小陀螺，`0x402` 的 5 字节为 `F4 01 06 FF 01`。零速帧为 `00 00 00 00 00`。

导航发送前应限制每轴速度在 ±2000 mm/s。只有符合 ID、帧类型和 DLC 的 `0x402` 会刷新底盘指令的 200 ms 计时。

## 1. USART6 物理层（切回 UART 时使用）

| 项目 | 参数 |
|---|---|
| 接口 | USART6（huart6） |
| 波特率 | 115200 |
| 数据位 / 停止位 / 校验 | 8 / 1 / 无（8N1） |
| 电平 | 3.3V TTL |
| 接线 | 底盘 RX = **PG9**（接雷达 TX），底盘 TX = **PG14**（接雷达 RX），共地 |
| 方向 | **单向**：雷达 → 底盘。底盘当前不向雷达回传任何数据 |

## 2. UART 帧格式

所有多字节字段均为**小端序（低位在前）**。

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 1 | SOF | 帧头，固定 `0xA5` |
| 1 | 2 | data_length | uint16，data 段字节数 |
| 3 | 1 | seq | 帧序号，底盘不校验，可自由填写（建议递增） |
| 4 | 1 | CRC8 | 对偏移 0~3 共 4 字节计算，见 §3 |
| 5 | 2 | cmd_id | uint16，见 §4 |
| 7 | data_length | data | 各命令的负载，见 §4 |
| 7+data_length | 2 | CRC16 | 对偏移 0 至 data 末尾（共 7+data_length 字节）计算，**低字节在前**，见 §3 |

- 帧总长 = data_length + 9 字节。
- 底盘按字节流解析：扫描 `0xA5` → 校验 CRC8 和长度 → 收齐整帧 → 校验 CRC16 → 分发。支持跨串口回调拼帧，也支持一次回调收到多帧。负载中的 `0xA5` 无需转义。
- 当前接收缓冲区可处理总长最多 128 字节的帧，即 `data_length ≤ 119`。

## 3. UART CRC 参数

两种 CRC 均使用反射算法（`refin = true`、`refout = true`、`xorout = 0`）。下方代码用低位先处理、右移的方式计算。

### CRC8
- 多项式：`0x31`（x⁸+x⁵+x⁴+1），右移实现使用 `0x8C`
- 初值：`0xFF`

### CRC16
- 多项式：`0x1021`（x¹⁶+x¹²+x⁵+1），右移实现使用 `0x8408`
- 初值：`0xFFFF`
- 发送顺序：校验值**低字节在前**

### Python 参考实现

```python
import struct

def crc8(data: bytes, init: int = 0xFF) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc

def crc16(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc

def pack_frame(cmd_id: int, data: bytes, seq: int = 0) -> bytes:
    header = bytes([0xA5]) + struct.pack('<H', len(data)) + bytes([seq])
    body = header + bytes([crc8(header)]) + struct.pack('<H', cmd_id) + data
    return body + struct.pack('<H', crc16(body))   # 低字节在前
```

C 侧参考实现：工程内 `BSP/algorithm/crc.c`（`Get_CRC8_Check_Sum` / `Get_CRC16_Check_Sum` / `Append_CRC16_Check_Sum`）。

## 4. UART 命令定义

### 4.1 cmd_id = 0x0401 自决策标志

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 4 | self_decision，uint32 |

哨兵协议中的"自决策"使能标志。**当前底盘仅存储，不产生任何行为。**

### 4.2 cmd_id = 0x0402 底盘控制（主要命令）

data_length = 9：

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 4 | vx | float32，底盘 x 向期望速度，m/s |
| 4 | 4 | vy | float32，底盘 y 向期望速度，m/s |
| 8 | 1 | flags | bit0：小陀螺（底盘自转）；bit1：云台扫描；bit2：超级电容 |

`vx` 和 `vy` 必须是有限值。当前排球车只执行 bit0，bit1 和 bit2 会被解析但不参与控制。

```python
def chassis_cmd(vx: float, vy: float, spin: bool = False,
                scan: bool = False, use_cap: bool = False, seq: int = 0) -> bytes:
    flags = (spin & 1) | ((scan & 1) << 1) | ((use_cap & 1) << 2)
    return pack_frame(0x402, struct.pack('<ffB', vx, vy, flags), seq)

# 实测零速帧（seq = 0，flags = 0）
assert chassis_cmd(0.0, 0.0) == bytes.fromhex(
    'A5 09 00 00 4D 02 04 00 00 00 00 00 00 00 00 00 79 46'
)
```

### 4.3 cmd_id = 0x0403 云台控制

data_length = 4：

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 4 | gimbal_yaw，float32，大云台 yaw 期望角度 |

当前排球车的处理函数未写入该角度，收到命令后无云台动作。

## 5. 坐标系约定（CAN 和 UART 共用）

- `vx` / `vy` 按**随车转动的底盘坐标系**解释。三轮解算的航向参数固定传 0，固件没有世界系转底盘系的航向补偿。导航若生成世界系速度，应先转换到底盘系再发送。
- 实车轴向由轮位、轮序和电机正方向决定。分别发送 `(vx=+0.5, vy=0)` 和 `(vx=0, vy=+0.5)`，观察并记录车体的 +x 和 +y，再验证反向。与导航组按实测结果对齐轴向。
- `CHASSIS_MAX_V` 为 **2.0 m/s**，适用于各平移分量。导航侧应将每轴指令限制在 **[-2.0, +2.0] m/s**。固件对越界值做循环折返，例如 `+2.5` 会变成 `-1.5`。速度指令随后经过一阶低通滤波。

## 6. 行为与限制（CAN 和 UART 共用）

1. 协议没有 `vw` 字段。UPC 模式通过 `flags.bit0` 请求小陀螺，仍可同时下发 `vx` / `vy`。固件自转设定 `SPIN_VW_SPEED` 为 **+1.8 rad/s**，实际车体转速和方向以实测为准。
2. **NUC 超时为 200 ms**。在 UPC 档下，首次收到有效 `0x0402` 前，或距最后一条有效 `0x0402` 已达 200 ms 时，三轴速度目标为零，小陀螺请求关闭。底盘通过现有滤波和速度 PID 减速。无效帧和其他命令都不会刷新计时。
3. **收到下一条有效 `0x0402` 会自动恢复响应**，条件是遥控器仍在线且 S1 仍在 UPC 档。新帧若有非零速度或开启小陀螺，车可能重新运动。正常联调请周期发送，建议 100–200 Hz。主动停车时继续发送零速帧并清除小陀螺标志。
4. 遥控器 S1 上档（值 1）为手动控制，中档（值 3）为 UPC，下档（值 2）为停机。下档及遥控器超过 1.5 秒未收到数据后的离线状态均将速度目标设为零，沿用滤波和速度闭环。
5. 底盘当前无 NUC 状态回传。使用 UART 时，`seq` 可固定填写。

## 7. 联调建议顺序

1. S1 置于中档，由导航发送小速度（如 0.3 m/s）单轴指令，确认 ±x、±y 方向。
2. 测试 `flags.bit0`，记录实际自转方向和转速。
3. 低速时停发超过 200 ms，观察底盘开始减速及小陀螺请求清零。先恢复发送零速帧，再发低速有效帧，确认底盘自动响应。
4. 联调时由操作手掌握遥控器。上档且摇杆回中可人工接管，下档可进入停机模式。
