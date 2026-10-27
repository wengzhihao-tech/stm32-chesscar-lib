# 公开参考资料索引

本次调研收集的可参考实现与技术文章，按模块分类。链接均为公开可访问页面。

---

## TB6612FNG 电机驱动

- **TB6612FNG 官方数据手册**（真值表、fPWM 上限 100kHz、VM 2.5~13.5V、
  每通道 1.2A 平均 / 3.2A 峰値、内置热关断与死区时间）
  <https://www.tme.eu/Document/f69c44f5b284761d41a1fa952256eb8d/TB6612FNG.pdf>
  <https://tams.informatik.uni-hamburg.de/lehre/2019ss/vorlesung/es/doc/tb6612fng.pdf>
- **TB6612FNG 引脚与接线说明**（STBY 必须拉高、方向脚同电平不转、A/B 并联驱动大电流电机）
  <https://www.tinkered.ai/components/tb6612fng-motor-driver>
  <https://www.sparkfun.com/sparkfun-motor-driver-dual-tb6612fng-1a.html>
- **PWM 与 TB6612FNG 驱动电机（两轮自平衢小车实战指南）**
  含完整 CubeMX 配置步骤、24kHz PWM 的 PSC/ARR 取値、占空比公式
  <https://c.miaowlabs.com/A25.html>
- **STM32Cube HAL 库利用 PWM 控制电机转动（TB6612）**
  对比 L298N 的热耗与外围电路差异
  <https://blog.csdn.net/weixin_52916289/article/details/131855523>
- **STM32CubeMX TB6612 直流电机驱动**
  <https://developer.aliyun.com/article/1566444>
- **STM32 PWM 与 `__HAL_TIM_SET_COMPARE` 用法**（多通道需分别 Start）
  <https://controllerstech.com/pwm-in-stm32>

## 编码器测速

- **野火开源电机教程 · 编码器的使用**（4 倍频原理、600PPR → 2400 等效分辨率、
  4 倍频扩展 M 法测速下限）
  <https://github.com/Embedfire-motor/ebf_motor_tutorial_stm32f407/blob/master/improve_part/use_of_encoder.rst>
- **STM32 Timer Encoder Mode 教程**（`HAL_TIM_Encoder_Start`、`__HAL_TIM_GET_COUNTER`、
  CubeMX 配置、常见坑：方向反转/计数不稳/计数器回绕用有符号整型）
  <https://controllerstech.com/incremental-encoder-with-stm32>
  <https://deepbluembedded.com/stm32-timer-encoder-mode-stm32-rotary-encoder-interfacing>
- **STM32 Timers: Read Quadrature Encoders**（`__HAL_TIM_IS_TIM_COUNTING_DOWN` 判方向、
  Counter Period 设为 PPR×4）
  <https://blog.embeddedexpert.io/stm32-timers-applications-read-quadrature-encoders>
- **编码器测速原理及 STM32 编码器模式**（M 法 / T 法 / M-T 法对比）
  <http://dev.guyuehome.com/wap/detail?id=1825491257694941186>
- **实战经验：在 STM32 中实现电机测速的方法介绍**
  <https://mcu.eetrend.com/content/2025/100589256.html>
- **STM32CubeMX 配置 HAL 库编码器测速**（速度换算公式）
  <https://yuchanghui.top/2023/08/24/STM32CubeMx%E9%85%8D%E7%BD%AEHAL%E5%BA%93%E7%BC%96%E7%A0%81%E5%99%A8%E6%B5%8B%E9%80%9F>

## PID 与串级控制

- **野火 · PID 算法的通俓解说**（离散化推导、位置式需积分限幅+输出限幅、
  增量式只需输出限幅、两者适用对象对比）
  <https://doc.embedfire.com/motor/motor_tutorial/zh/latest/improve_part/PID_detailed.html>
  <https://github.com/Embedfire-motor/ebf_motor_tutorial_stm32f407/blob/master/improve_part/PID_detailed.rst>
- **野火 · 直流电机速度环控制实现**（位置式与增量式两套例程、定时中断执行 PID 的编程要点）
  <https://doc.embedfire.com/motor/motor_tutorial/zh/latest/improve_part/dc_motor_speed_loop_control.html>
- **野火 · 直流电机位置环控制实现**（增量式公式、误差死区处理）
  <https://doc.embedfire.com/mcu/renesas/motor/zh/latest/doc/improve_part/dc_motor_pos_loop_control.html>
- **野火 · 位置速度双环控制实现**（外环输出作为内环输入、目标速度上限鼗位）
  <https://doc.embedfire.com/motor/motor_tutorial/zh/latest/improve_part/step_motor_double_loop_control.html>
- **KeyL-SJ/BalanceCar**（串级 PID：速度环 PI + 直立环 PD，速度环加一阶低通减小对直立环干扰）
  <https://github.com/KeyL-SJ/BalanceCar>

## 串口 DMA 与不定长接收

- **MaJerle/stm32-usart-uart-dma-rx-tx** —— 该主题最权威的参考实现
  覆盖"接收长度未知"的 UART+DMA 接收与非阻塞发送
  <https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx>
- **ST 官方中文论坛 · STM32 HAL 库串口空闲中断最新用法**
  `HAL_UARTEx_ReceiveToIdle` / `_IT` / `_DMA` 三种模式与
  `HAL_UARTEx_RxEventCallback` 的用法
  <https://shequ.stmicroelectronics.cn/thread-637349-1-1.html>
- **STM32_USART_DMA_RX**（IDLE 线 + DMA 半满/全满中断方案的优缺点、
  两个中断向量须设相同抓占优先级）
  <https://github.com/1847123212/STM32_USART_DMA_RX>
- **UART IDLE Line interrupt with DMA**（寄存器层面 `USART_CR1_IDLEIE`、`USART_CR3_DMAR`）
  <https://blog.embeddedexpert.io/working-with-stm32-and-uart-part-6-idle-line-interrupt-with-dma>
- **STM32 用 DMA+IDLE 中断方式串口接收不定长数据**（NORMAL 模式下需重启 DMA）
  <https://www.guyuehome.com/wap/detail?id=1825502792446177282>
- **UART 任意数据的收发**（需关闭 DMA 传输过半中断）
  <http://docs.baud-dance.com/docs/stm32/example/UART_ECHO>
- **stm32H7 UART 使用与 HAL 代码分析**（`HAL_UART_IRQHandler` 的错误处理路径）
  <https://fealonelei.github.io/2024/05/29/stm32H7-serial-port.html>

## SSD1306 OLED

- **afiskon/stm32-ssd1306** —— 约 1.1k star，MIT，支持 SSD1306/SH1106/SH1107/SSD1309，
  I2C 与 4 线 SPI；需要图形功能时推荐直接用它
  <https://github.com/afiskon/stm32-ssd1306>
- **4ilo/ssd1306-stm32HAL** —— 上述库的前身
  <https://github.com/4ilo/ssd1306-stm32HAL>
- **STM32 SSD1306 OLED I2C — HAL & CubeMX 教程**（4 线接线、CubeMX I2C 配置）
  <https://controllerstech.com/oled-display-using-i2c-stm32>
- **stm32 HAL 库硬件 IIC 使用 OLED（SSD1306）**
  <https://blog.csdn.net/weixin_44065323/article/details/129332036>
- **STM32-OLED-I2C-HAL-Coding-Method**（STM32F446RE + 128×64 示例）
  <https://github.com/DanielRajChristeen/STM32-OLED-I2C-HAL-Coding-Method>

## 灰度 / 光电循迹传感器

- **8 路灰度教学工程** —— 本库 `line_sensor.c` 加权平均法的主要参考
  权重 −7/−5/−3/−1/1/3/5/7、`error = 权重和 / 命中数`、
  全黑停车与全白沿用上次偏差、10ms 采样 / 20ms 控制双节拍、
  以及"微分项必须用控制周期的 previousError"这一关键细节
  <https://www.cnblogs.com/zw-awa/p/23366990>
- **灰度传感器校准：从原理到实践**（先取黑白极限 ADC 读数再归一化到统一范围）
  <https://wenku.csdn.net/column/nma9n47z1tl>
- **STM32 循迹小车系列教程（三）· 使用灰度传感器循迹**
  <http://dev.guyuehome.com/detail?id=1825500266942492673>
- **七路灰度传感器巡线算法实现**（传感器阵列排列与 ADC 采样）
  <https://bbs.csdn.net/weixin_29191081/article/details/100177473>
- **STM32 智能小车 PID 循迹算法优化：从左右摇摆到平滑巡线**
  <https://bbs.csdn.net/weixin_29166013/article/details/100179029>

---

## 2026-10-26 增量：运动斜坡相关调研

本节是新增 `common/motion_ramp.*` 时查证的资料。
上文已列出的条目不重复，只标注它在本次设计里起了什么作用。

### 软启动斜坡的公开实现

- **baihao774/stm32-tb6612-driver**（STM32F103 标准外设库，TIM3_CH3 单 A 通道）
  README 明确写出「**升速软启动斜坡：每个控制周期占空比递增 5%，
  抑制电机启动冲击电流**」，主循环按固定 20ms 调用 `TB6612_Update()` 推进斜坡。
  另外两点与本库取値一致，可互为旁证：**20kHz PWM**、
  **占空比分辨率 1/3600**（对应 72MHz/3600）；
  以及三种停止方式的明确区分（刹车 / 滑行停止 / 待机）。
  <https://github.com/baihao774/stm32-tb6612-driver>

  **本库的不同做法**：它斜坡的是**开环占空比**，本库斜坡的是**目标速度**，
  挂在速度闭环之前。若照抄到速度环之后去限制 PWM 变化率，
  等于给闭环串了一阶滞后，会改变已调好的 PID 增益含义。
  详见 [`MOTION_RAMP.md`](MOTION_RAMP.md)。

### 低速不可分辨下限的依据

- **实战经验：在 STM32 中实现电机测速的方法介绍**（已在『编码器测速』节列出）
  本次重点用到它的两个结论：
  **① M 法分辨率与转速无关**，要提高只能增大每圈脚冲数 Z 或采样周期 Ts；
  **② M 法误差率与测定时间内脚冲数 M1 成反比**，转速越低 M1 越小、误差率越大，
  所以 M 法适合高速、T 法适合低速。文中附了 1460~2920rpm、1460~292rpm、
  19~35rpm 三档模拟测试曲线，结论是「高速下 M 法效果更好，
  低速情况下 M 法测速偏差大，T 法更适合低速测速」。
  本库是纯 M 法（固定 10ms 读计数差），因此低速只能靠 `minMoveCmps` 下限回避，
  这是原理限制而非调参问题。
  文中还给出可直接照抄的硬件方案：TIM3 编码器接口 4x 计数 → TIM3_TRGO
  作为 TIM2 的捕获与复位触发源 → TIM6 做 ms 级中断，
  在中断里同时算 M 法（读 TIM3_CNT 差値）和 T 法（读 TIM2_CCR1）。
  将来若确实需要低速精确闭环，这是正解。
  <https://mcu.eetrend.com/content/2025/100589256.html>

### 同类整车工程（可对照参数与工程组织）

- **09-25/m0-line-tracing-car**（MSPM0G3507 + 8 路灰度 + 编码器 + ICM42688 +
  SSD1306 + TB6612/DRV8701 可选）
  最有价値的一条旁证：该工程同样用 **20kHz 电机 PWM + 10ms 主控制周期**，
  与本库 `CAR_PWM_FREQUENCY_HZ` / `CAR_SPEED_LOOP_PERIOD_MS` 完全一致。
  工程组织也同思路：参数集中在单个 `car_trace.h`（对应本库
  `car_physical_config.h`）；README 明确声明「仓库中的参数用于当前底盘与赛道，
  移植到其它底盘前需要重新标定」，并建议改参数前先记录原値、
  **先抬起车轮进行方向和急停测试**——与 [`TUNING_GUIDE.md`](TUNING_GUIDE.md)
  第 0 步「全程架空直到第 4 步」的要求一致。
  <https://github.com/09-25/m0-line-tracing-car>
- **5ee511/MSPM0G3507-Car**（八路灰度直角弯巡线，TB6612 + 编码器速度 PI + 巡线 PD）
  其控制分工与本库 `speed_control`（内环 PI）+ `line_follow`（外环 PD）完全对应，
  README 标注基础直线、普通弯道、左右直角弯均已实车验证。
  <https://github.com/5ee511/MSPM0G3507-Car>
- **Heximiao/m0test**（MSPM0G3507 + TB6612 + 11PPR 霍尔编码器双轮速度闭环）
  公开了一组可对照的量级：最大线速度 **100mm/s**、最大角速度 **120deg/s**；
  速度目标单位是「每 20ms 控制周期的编码器增量」，默认 `BASE = 20 counts/20ms`，
  原地 90° 转弯按约 275 个 count 换算、转弯速度 15 counts/20ms。
  可作为「自己的参数是否离谱」的粗检标尺，但**不能直接比计数**：
  它是 11PPR / 20ms，本车是 13PPR×4 / 10ms，必须各自折算成 cm/s 再比。
  设计取舍上値得一提：它直接用 counts/周期 表达速度目标，省掉换算误差；
  本库统一用 cm/s（见 `chesscar_common.h` 的单位约定），
  代价是多一层换算，好处是上层算法读起来有物理意义、换平台不用改。
  <https://github.com/Heximiao/m0test>

### 顺带记录（本次未采用，留作备选）

- **Prry/stm32-uart-dma**（F030/F103/F407/H743 四平台实测，标准库与 HAL/LL 并存）
  收发双向环形缓冲 + 不定长接收，三条接收路径（DMA 半满中断 /
  传输完成中断 / 空闲中断）并存。本库 `uart_link.c` 目前只用
  「空闲中断 + 关半传输中断」，若将来提高波特率可参考其半满双缓冲方案。
  <https://github.com/Prry/stm32-uart-dma>
- **U8g2 移植到 STM32**（硬件 I2C / 硬件 SPI / 软件 SPI 三种 callback 写法）
  本库 `oled_ssd1306.*` 只有 8x16 ASCII 字库，够显示调参数値。
  若现场需要画棋子分布图或状态条，U8g2 只需实现 `u8x8_byte_*` 与
  `u8x8_gpio_and_delay` 两个回调即可接入。
  代价：整屏缓冲占约 1KB RAM，且刷新仍是阻塞 I2C，
  [`PITFALLS.md`](PITFALLS.md) 第 20 条（整屏刷新不能放进控制中断）同样适用。
  <https://controllerstech.com/how-to-port-u8g2-graphic-lib-to-stm32>
  <https://www.cnblogs.com/frozencandles/p/16358483.html>

---

## 2026-10-27 增量：ADC 多通道 DMA + UART HAL 版本迁移

本节是新增 `stm32/adc_line_sensor.*` 时查证的资料，以及 UART 参考资料的补充。

### ADC 多通道 DMA 扫描（灰度探头模拟量版本）

- **STM32 ADC Multi-Channel Circular DMA**（CubeMX 配置步骤截图）
  核心配置：Scan Conversion Mode + Continuous Conversion Mode + DMA Circular + Half Word；
  指出 F103 上无独立"DMA Continuous Requests"开关（F4 以上才有），
  Circular 模式下 ADC 自动持续请求 DMA。
  本库 `stm32/adc_line_sensor.c` 的 8 路版本以此为基础扩展。
  <https://controllerstech.com/stm32-adc-multiple-channels-circular-dma>
- **STM32 ADC and DAC with DMA — stm32world wiki**
  双缓冲方案：`HAL_ADC_ConvHalfCpltCallback`（半满）与
  `HAL_ADC_ConvCpltCallback`（全满）交替处理前后半块缓冲，
  可进一步降低快照延迟（本库单缓冲快照已足够 10ms 控制周期）。
  三通道扩展至 8 通道只需改 `Number of Conversions` 和 DMA buffer 大小。
  <https://stm32world.com/wiki/STM32_ADC_and_DAC_with_DMA>

### UART HAL 版本迁移提示

- **STM32 Variable-Length UART RX: Receive-to-IDLE + Circular DMA**
  重点：STM32CubeF4/HAL1 使用 `HAL_UARTEx_ReceiveToIdle_DMA()`（含 Ex），
  ST 的 HAL1→HAL2 迁移文档中对应函数改名为 `HAL_UART_ReceiveToIdle_DMA()`（去掉 Ex）；
  架构模式相同，符号不同。若在较新 HAL 版本中报"未定义标识符"，
  先确认 HAL 版本，再按需换用新名（或在旧版工程里保持原名）。
  <https://pcbcool.com/technical-guides/stm32-variable-length-uart-rx-using-receive-to-idle-and-circular-dma>
- **STM32 UART Idle Line: Receive Data via Interrupt & DMA**（三模式对比）
  IT 模式（小帧）/ 单次 DMA 模式（KB 级）/ Circular DMA 模式（持续流）
  的适用场景与典型 buffer ownership 模型（head/tail pointer）；
  所有 STM32 家族（F0/F1/F4/F7/H7/G0/G4/L4）均支持 `HAL_UARTEx_ReceiveToIdle_DMA()`，
  若报编译错误先检查 NVIC 是否使能了 UART 全局中断。
  <https://controllerstech.com/stm32-uart-5-receive-data-using-idle-line>
