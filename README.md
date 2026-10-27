# stm32-chesscar-lib

面向**第十五届全国大学生光电设计竞赛赛题一「智能下棋小车」**的
STM32F103 双轮差速小车库。

库分两层：

- **`common/`** —— 平台无关的算法层，不含任何芯片寄存器访问，
  可在 PC 上编译并运行单元测试。
- **`stm32/`** —— 基于 STM32 HAL 的硬件抄象层（PWM、编码器、I2C、UART DMA、ADC DMA）。

这样分层的原因很实际：算法逻辑在主机上跑一轮自测，比每次烧进单片机看 OLED 快得多。

---

## 快速开始

```bash
git clone https://github.com/wengzhihao-tech/stm32-chesscar-lib.git
```

1. 把 `common/` 和 `stm32/` 加入工程的 include 路径。
2. 根据实际接线修改 `stm32/bsp_config.h`。
3. 按实测値修改 `common/car_physical_config.h`（**这一步不能跳**）。
4. 参考 `examples/app_chesscar.c` 组织主循环与中断回调。
5. **先读 [`docs/TUNING_GUIDE.md`](docs/TUNING_GUIDE.md) 再上车。**

---

## 模块构成

| 文件 | 职责 | 关键点 |
|---|---|---|
| `common/chesscar_common.h` | 共享类型、限幅、低通、死区补偿 | 统一单位：cm / cm/s / deg / ms |
| `common/car_physical_config.h` | **机械与电气参数唯一入口** | 轮径、减速比、PPR、死区、PWM 频率 |
| `common/pid_controller.*` | 位置式 + 增量式 PID | 条件积分抗饱和，不止积分限幅 |
| `common/encoder_math.*` | 脚冲 ↔ 速度 ↔ 里程换算 | 提供速度量化步长，用于判断低速可分辨下限 |
| `common/speed_control.*` | 双轮速度闭环 | 前馈 + 死区补偿 + 方向/超速自检 |
| `common/motion_ramp.*` | 目标速度斜坡 | 软启动/软停、**先过零再换向**、低速不可分辨下限 |
| `common/line_sensor.*` | 8 路探头归一化与加权偏差 | 本赛题可降级用于边界/压线检测 |
| `common/line_follow.*` | 循迹外环 PD | **本赛题用不上**，保留以备循迹练习 |
| `common/link_protocol.*` | 帧头 + CRC16 二进制协议 | 逐字节状态机，可从粘包/丢字节恢复 |
| **赛题专用模块（↓ 以下为赛醘1 新增）** | | |
| `common/field_config.h` | **场地/规则/车辆常量唯一入口** | 含「待主办方确认」项的明确标注 |
| `common/pose_estimator.*` | 差速运动学死算 + 航向融合 | 支持陵镜/视觉修正，带场地越界检测 |
| `common/chess_target.*` | 棋子评分与目标选择 | 考虑跨分趋势与对手威胁 |
| `common/navigator.*` | 路径规划与推送对准 | 对手过近强制规避 |
| `common/match_fsm.*` | 比赛主状态机 | 待命→接近→对准→推送→得分→退出 |
| `common/vision_chess.*` | 视觉帧解析与置信度过滤 | 2cm 量化，低置信度直接丢弃 |

---

## 硬件抄象层

| 文件 | 对应外设 |
|---|---|
| `stm32/motor_tb6612.*` | TIM1 PWM + 方向 GPIO + STBY |
| `stm32/encoder_tim.*` | TIM2/TIM3 编码器模式（4 倍频） |
| `stm32/oled_ssd1306.*` | I2C1，128×64，8x16 ASCII 字库 |
| `stm32/uart_link.*` | USART1 + DMA + 空闲中断 |
| `stm32/tof_vl53l0x.*` | 多路 VL53L0X（简化实现） |
| `stm32/adc_line_sensor.*` | ADC1 + DMA Circular，8 路灰度探头采样 |

---

## PC 端自测

PC 端验证算法层：

```bash
cd test && make run        # 全部
cd test && make run-ramp   # 只跑运动斜坡
```

## 验证状态

- **PC 自测编译 14 个编译单元**（`common/` 11 个 .c + 3 个用例 main），
  在 `gcc -std=c99 -Wall -Wextra` 下零警告零错误通过。
  `common/` 共 12 个 .c，其中 `speed_control.c` 未进入 PC 自测（依赖硬件接口）；
  `stm32/` 的 6 个 .c 与 `examples/` 的 3 个 .c 不在 PC 自测范围内。
- **算法层 160 项断言全部通过**（基础 38 + 赛题专用 100 + 运动斜坡 22），
  含 CRC16-CCITT-FALSE 标准向量 `"123456789" → 0x29B1`，
  以及完整得分流程（接近→对准→推送→进门→记分→退出）的端到端验证。
- **硬件层尚未在实物 STM32 上验证**：`stm32/` 下的 HAL 调用序列经过编译与接口核对，
  但引脚分配、编码器方向、ToF 改址流程必须现场确认。
- `stm32/tof_vl53l0x.c` 是简化实现（多路组织 + 读数过滤），
  完整校准建议引入 ST 官方 ULD 驱动，详见赛题文档。
- `common/motion_ramp.*`（2026-10-26 新增）只验证了数値逻辑：
  斜坡步长、换向过零、低速下限、急停清零。
  **加/减速度、角速度变化率、低速下限、换向保持拍数这五个物理値仍需上车实测**
  ——数値逻辑正确不等于参数合适。标定方法见
  [`docs/MOTION_RAMP.md`](docs/MOTION_RAMP.md)。
- `stm32/adc_line_sensor.*`（2026-10-27 新增）只验证了编译；
  ADC Circular DMA 启动顺序（MX_DMA_Init 须先于 MX_ADC1_Init）、
  DMA 缓冲地址对齐、以及 8 路通道 Rank 顺序与探头编号的对应关系
  必须在实物 STM32 上用串口打印原始 ADC 値确认。

## 文档

- [`docs/COMPETITION_TASK1.md`](docs/COMPETITION_TASK1.md) —— **赛醘1 方案与硬件选型（先读这个）**
- [`docs/TUNING_GUIDE.md`](docs/TUNING_GUIDE.md) —— 标定与调参顺序（**上车前必读**）
- [`docs/PITFALLS.md`](docs/PITFALLS.md) —— 踩坑清单
- [`docs/MOTION_RAMP.md`](docs/MOTION_RAMP.md) —— 运动斜坡原理与标定（软启动/换向/低速下限）
- [`docs/PROTOCOL.md`](docs/PROTOCOL.md) —— 串口协议规格
- [`docs/REFERENCES.md`](docs/REFERENCES.md) —— 公开参考资料索引

## 许可

MIT
