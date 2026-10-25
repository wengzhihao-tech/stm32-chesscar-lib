# 串口协议规格

STM32 ↔ 上位机 / 视觉板。默认 **115200 8N1**。

## 帧格式

所有多字节字段为**小端**，使用固定宽度整数。

```
偏移  长度  字段      说明
0     1    SOF1      0xA5
1     1    SOF2      0x5A
2     1    type      消息类型
3     1    sequence  递增序号（溢出回绕），用于 ACK 与丢帧诊断
4     1    flags     bit0 请求ACK / bit1 是ACK / bit2 错误
5     1    length    payload 字节数，最大 32
6     N    payload
6+N   2    CRC16     CRC16-CCITT-FALSE，低字节在前
```

**CRC 覆盖范围**：`type`、`sequence`、`flags`、`length`、`payload`，
**不含**两个帧头字节。

**CRC 参数**：多项式 `0x1021`，初值 `0xFFFF`，结果不反转、不异或。
标准校验向量：`"123456789"` → `0x29B1`（本库自测已验证）。

最小帧 8 字节（无 payload），最大帧 40 字节。

## 消息类型

| 值 | 名称 | 方向 | payload |
|---|---|---|---|
| 0x01 | `HELLO` | 双向 | 无 |
| 0x02 | `HEARTBEAT` | 双向 | 无 |
| 0x10 | `CAR_STATUS` | STM32 → 上位机 | 见下 |
| 0x11 | `VISION_RESULT` | 视觉 → STM32 | 目标识别结果 |
| 0x13 | `LINE_RESULT` | 视觉 → STM32 | 黑线循迹结果 |
| 0x18 | `CHESS_MOVE` | 上位机 → STM32 | `[0]`起点格号 `[1]`终点格号 |
| 0x20 | `TASK_COMMAND` | 上位机 → STM32 | `[0]` 目标模式 |
| 0x21 | `COMMAND_ACK` | STM32 → 上位机 | 被确认的 sequence |
| 0x30 | `PID_TUNE` | 上位机 → STM32 | 见下 |
| 0x7F | `ERROR` | 双向 | 错误码 |

### `CAR_STATUS` (0x10)，14 字节

| 偏移 | 类型 | 字段 | 定标 |
|---|---|---|---|
| 0 | u8 | 当前模式 | 见 `App_CarMode` |
| 1 | u8 | 故障码 | 见 `SpeedControl_Fault` |
| 2 | u8 | 探头掩码 | bit0 = 最左 |
| 3 | u8 | 保留 | — |
| 4 | i16 | 左轮速度 | ×10，单位 cm/s |
| 6 | i16 | 右轮速度 | ×10，单位 cm/s |
| 8 | i16 | 循迹偏差 | ×100 |
| 10 | i32 | 累计里程 | ×10，单位 cm |

> 全部按整数定标传输，避免浮点在不同上位机上解析不一致。

### `PID_TUNE` (0x30)，13 字节

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u8 | 环编号：0 = 速度环，1 = 循迹环 |
| 1 | f32 | Kp |
| 5 | f32 | Ki |
| 9 | f32 | Kd |

**务必在车停住时改参数。** 接收端 `PID_SetGains()` 会自动清积分。

## 解析器行为

- 逐字节状态机，可从粘包、拆包、丢字节中恢复。
- 复位时若当前字节恰好等于 `SOF1`，直接当作下一帧起点，加快重同步。
- `length > 32` 立即拒绝并计入 `lengthErrorCount`。
- CRC 不匹配则丢弃整帧并计入 `crcErrorCount`，**不会**触发上层回调。

## 诊断计数

| 接口 | 含义 |
|---|---|
| `UartLink_GetValidFrameCount()` | CRC 正确的完整帧数 |
| `UartLink_GetCrcErrorCount()` | CRC 错误数，持续增长 = 线路干扰或波特率不匹配 |
| `UartLink_GetOverrunCount()` | 环形缓冲溢出数，非零 = 主循环调用 `UartLink_Poll()` 太慢 |
