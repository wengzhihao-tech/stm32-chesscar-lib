# stm32-chesscar-lib

面向**第十五届全国大学生光电设计竞赛 赛题1「智能车光电象棋对抗」**的
STM32 可复用代码库（HAL 库 + STM32CubeMX）。

本库从本机已上车验证的 `smartcar` 工程（TI MSPM0 平台）提取算法层，重写硬件层为
STM32F1/F4 的 HAL 实现，并补充了编译验证与 PC 端算法自测。

> ## ⚠ 本赛题不是循迹赛
>
> 赛题是 3m×2m **开放场地**推棋子进球门，场上**没有引导线**。
> 核心难点是「自主定位 + 棋子识别 + 目标决策」，不是跟线。
>
> 因此 `line_follow`（循迹外环）在本赛题**用不上**（保留以备循迹练习），
> 比赛主线由 `navigator`（点到点导航）+ `match_fsm`（比赛状态机）驱动。
>
> **赛题方案与硬件选型请先读 [`docs/COMPETITION_TASK1.md`](docs/COMPETITION_TASK1.md)。**

## 为什么这样分层

```
common/   平台无关算法层 —— 不含任何寄存器访问，可在 PC 上编译与单元测试
stm32/    STM32 HAL 硬件层 —— 只做寄存器与外设操作，不含控制策略
examples/ 应用层 —— 节拍调度、模式机、遥测
test/     PC 端自测 —— 用主机 gcc 验证算法，不必烧板
```

这样分层的实际好处：**调 PID 不用烧单片机**。`make -C test run` 几秒就能验证
限幅、抗饱和、偏差符号、协议解析是否正确；上车只需要标定物理参数。

换平台（比如换回 MSPM0 或换 GD32）时，`common/` 一行不用改，只重写 `stm32/`。

## 模块清单

| 文件 | 作用 | 关键点 |
|---|---|---|
| `common/chesscar_common.h` | 共享类型、限幅、低通、死区补偿 | 统一单位：cm / cm/s / deg / ms |
| `common/car_physical_config.h` | **机械与电气参数唯一入口** | 轮径、减速比、PPR、死区、PWM 频率 |
| `common/pid_controller.*` | 位置式 + 增量式 PID | 条件积分抗饱和，不止积分限幅 |
| `common/encoder_math.*` | 脉冲 ↔ 速度 ↔ 里程换算 | 提供速度量化步长，用于判断低速可分辨下限 |
| `common/speed_control.*` | 双轮速度闭环 | 前馈 + 死区补偿 + 方向/超速自检 |
| `common/line_sensor.*` | 8 路探头归一化与加权偏差 | 本赛题可降级用于边界/压线检测 |
| `common/line_follow.*` | 循迹外环 PD | **本赛题用不上**，保留以备循迹练习 |
| `common/link_protocol.*` | 帧头 + CRC16 二进制协议 | 逐字节状态机，可从粘包/丢字节恢复 |
| **赛题专用模块（↓ 以下为赛题1 新增）** | | |
| `common/field_config.h` | **场地/规则/车辆常量唯一入口** | 含「待主办方确认」项的明确标注 |
| `common/pose_estimator.*` | 开放场地位姿估计 | 里程计 + IMU + **80mm 挡板贴边绝对校正** |
| `common/chess_target.*` | 棋子表管理与目标决策 | 权衡分值/推送距离/风险，不是简单推最近的 |
| `common/navigator.*` | 点到点导航 | 大角度先转再走；**对手过近强制规避**（规则红线） |
| `common/match_fsm.*` | 比赛状态机 | 搜索→接近→对准→推送→记分→退出，每状态带超时 |
| `common/vision_chess.*` | 视觉棋子上报解析 | 协议 0x14；**等级允许未知**（汉字识别不可靠） |
| `stm32/tof_vl53l0x.*` | 多路 ToF 测距管理 | 含 **I2C 地址冲突改址流程**（必踩的坑） |
| `stm32/bsp_config.h` | **硬件映射唯一入口** | 内含完整 CubeMX 勾选清单 |
| `stm32/motor_tb6612.*` | TB6612FNG 双电机驱动 | 换向前先清 PWM，防穿通电流 |
| `stm32/encoder_tim.*` | 定时器编码器模式测速 | 16 位计数器溢出累计为 32 位 |
| `stm32/uart_link.*` | 串口 DMA 空闲中断收发 | 环形缓冲 + 错误后自动重启接收 |
| `stm32/oled_ssd1306.*` + `oled_font.h` | OLED 调试显示 | 自带 95 字符 8x16 ASCII 字库 |
| `examples/app_chesscar.*` | 主应用与节拍调度 | 1ms 中断只置标志，运算在主循环 |
| `examples/hal_callbacks.c` | HAL 回调统一挂接 | 避免 CubeMX 重新生成后漏挂回调 |

## 快速开始

0. **先读 [`docs/COMPETITION_TASK1.md`](docs/COMPETITION_TASK1.md)**，
   确认赛题待定参数（球门尺寸、棋子分值表、比赛时长）并更新 `common/field_config.h`。
1. 用 CubeMX 按 `stm32/bsp_config.h` 顶部注释里的清单配置外设并生成工程。
2. 把 `common/`、`stm32/`、`examples/` 加入工程源文件与 Include Path。
3. 按 `examples/main_reference.c` 的说明，在 `main.c` 的 USER CODE 区加三行调用。
4. **先做台架标定**，再上地跑：见 [`docs/TUNING_GUIDE.md`](docs/TUNING_GUIDE.md)。

PC 端验证算法层：

```bash
cd test && make run
```

## 验证状态

- **17 个编译单元**在 `gcc -std=c99 -Wall -Wextra` 下零警告零错误通过（用 HAL 桩替换芯片头文件）。
- **算法层 136 项断言全部通过**（基础 36 + 赛题专用 100），
  含 CRC16-CCITT-FALSE 标准向量 `"123456789" → 0x29B1`，
  以及完整得分流程（接近→对准→推送→进门→记分→退出）的端到端验证。
- **硬件层尚未在实物 STM32 上验证**：`stm32/` 下的 HAL 调用序列经过编译与接口核对，
  但引脚分配、编码器方向、ToF 改址流程必须现场确认。
- `stm32/tof_vl53l0x.c` 是简化实现（多路组织 + 读数过滤），
  完整校准建议引入 ST 官方 ULD 驱动，详见赛题文档。

## 文档

- [`docs/COMPETITION_TASK1.md`](docs/COMPETITION_TASK1.md) —— **赛题1 方案与硬件选型（先读这个）**
- [`docs/TUNING_GUIDE.md`](docs/TUNING_GUIDE.md) —— 标定与调参顺序（**上车前必读**）
- [`docs/PITFALLS.md`](docs/PITFALLS.md) —— 踩坑清单
- [`docs/PROTOCOL.md`](docs/PROTOCOL.md) —— 串口协议规格
- [`docs/REFERENCES.md`](docs/REFERENCES.md) —— 公开参考资料索引

## 许可

MIT
