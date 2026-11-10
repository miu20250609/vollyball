# NUC 雷达 ↔ 底盘串口通信协议

> 适用对象：RoboVolley 工程（sentry26-robovolley26），底盘 STM32F407 C 板
> 协议来源：sentry26-main 哨兵工程 NUC 协议，底盘侧实现见 `Module/NUC/nuc.c`
> 版本：v1.0（2026-11-02）

## 1. 物理层

| 项目 | 参数 |
|---|---|
| 接口 | USART6（huart6） |
| 波特率 | 115200 |
| 数据位 / 停止位 / 校验 | 8 / 1 / 无（8N1） |
| 电平 | 3.3V TTL |
| 接线 | 底盘 RX = **PG9**（接雷达 TX），底盘 TX = **PG14**（接雷达 RX），共地 |
| 方向 | **单向**：雷达 → 底盘。底盘当前不向雷达回传任何数据 |

## 2. 帧格式

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
- 底盘按字节流解析：扫描 `0xA5` → 校验 CRC8 → 校验 CRC16 → 分发。data 中出现 `0xA5` 无需转义（CRC 会滤掉误匹配）。
- data_length 建议 ≤ 100 字节。

## 3. CRC 参数（务必照此实现）

两种 CRC 都是 **MSB-first、非反射（refin/refout = false）、xorout = 0**，与常见在线工具默认不同，请按参数手动实现或直接使用下方代码。

### CRC8
- 多项式：`0x31`（x⁸+x⁵+x⁴+1）
- 初值：`0xFF`

### CRC16
- 多项式：`0x1021`（x¹⁶+x¹²+x⁵+1，即 CRC-16/CCITT-FALSE）
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
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def crc16(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def pack_frame(cmd_id: int, data: bytes, seq: int = 0) -> bytes:
    header = bytes([0xA5]) + struct.pack('<H', len(data)) + bytes([seq])
    body = header + bytes([crc8(header)]) + struct.pack('<H', cmd_id) + data
    return body + struct.pack('<H', crc16(body))   # 低字节在前
```

C 侧参考实现：工程内 `BSP/algorithm/crc.c`（`Get_CRC8_Check_Sum` / `Get_CRC16_Check_Sum` / `Append_CRC16_Check_Sum`）。

## 4. 命令定义

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

```python
def chassis_cmd(vx: float, vy: float, spin: bool = False,
                scan: bool = False, use_cap: bool = False, seq: int = 0) -> bytes:
    flags = (spin & 1) | ((scan & 1) << 1) | ((use_cap & 1) << 2)
    return pack_frame(0x402, struct.pack('<ffB', vx, vy, flags), seq)
```

### 4.3 cmd_id = 0x0403 云台控制

data_length = 4：

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 4 | gimbal_yaw，float32，大云台 yaw 期望角度 |

**当前本车无云台，仅存储，不产生任何行为。**

## 5. 坐标系约定（联调前必须实测对齐）

- 底盘坐标系随**上电时刻**车身朝向固定，不随车旋转。
- ⚠️ **代码内部对"前进方向"存在矛盾**：`APP/robovolley.c` 第 246 行注释写"向前为 y 正方向"，但运动学解算矩阵（`Omni_Kinematics_Resolve`）与遥控器映射（vx = 左杆前后）推导出的前进方向是 **+x**。**以 +x = 车头前进、+y = 车身左侧（右手系，逆时针自转为正）为准，联调时务必按下方方法实测确认。**
- `vx` / `vy` 是**底盘坐标系下的期望速度**（m/s）。若雷达有自己的位姿/安装朝向，请由雷达侧完成坐标变换后再下发，底盘不做任何旋转变换。
- 底盘对 vx/vy 钳位到 **±4 m/s**（超出部分直接截断，不缩放），并对速度做一阶低通滤波（阶跃指令会被平滑，属正常现象）。
- **实测确认方法**：发 (vx=+0.5, vy=0)，观察车沿哪个方向走，该方向即 +x；再发 (vx=0, vy=+0.5) 确认 +y。若实测 +x 不是车头方向，请以实测为准并与电控同步修正。

## 6. 行为与限制（雷达侧须知）

1. **协议中不存在 vw 字段**，不能直接下发自转角速度。底盘自转只能通过 flags.bit0（小陀螺）触发，转速为固定值 +0.1 rad/s，且小陀螺模式下 vx/vy 仍照常生效。
2. **底盘无雷达超时保护**：雷达停止发送时，底盘会一直保持最后一次收到的速度。请务必**周期性发送**（建议 100~200 Hz，与哨兵原版一致），停车时发送全零帧，不要在需要停车时静默停发。
3. 控制权生效条件：遥控器 s1 拨杆位于**中间档**（DBUS 拨杆值 = 3）时进入上位机（UPC）模式，雷达数据生效；拨到其他档位立即由遥控器接管。
4. 底盘不回传任何数据；如需底盘状态（电量/当前速度等）另行协商扩展。
5. seq、以及 0x0401/0x0403 命令当前无实际作用，可按协议发送但勿依赖其行为。

## 7. 联调建议顺序

1. 雷达侧发 0x0402 小速度（如 0.3 m/s）单轴测试，确认 ±x、±y 方向与预期一致（不一致请检查坐标系约定 §5）；
2. 测试 flags.bit0 小陀螺；
3. 测试中途停发 3 秒，观察底盘会保持最后速度（预期行为），确认雷达侧零速帧逻辑；
4. 全程保留一名操作手在遥控器旁，异常时拨杆切回 RC 档急停。
