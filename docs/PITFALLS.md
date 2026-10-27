# 踩坑清单

按模块整理，含本工程已经处理掉的坑和需要你自己注意的坑。

---

## PWM 与电机

**1. CCR 的含义随计数模式而变（跨平台移植重灰区）**
STM32 通用定时器默认向上计数 + PWM Mode 1，`CCR` 就是高电平计数，
duty 越大占空比越高。而本工程早先的 MSPM0 版本用向下计数，
`CCR` 存的是"周期末尾剩余计数"，必须写 `SPEED_MAX - duty`。
**照抄那段公式到 STM32 会导致"低速命令反而接近满占空比"，极其危险**。
本库已在 `motor_tb6612.c` 的 `Motor_SetPwm()` 注释中标明。

**2. 多通道 PWM 必须分别 Start**
`HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2)` 是错的，
通道宏不能按位或。必须每个通道调一次。

**3. TB6612 的 STBY 不拉高则完全不动**
`STBY = L` 时两路输出都是高阻，电机一点反应没有，很容易误判成 PWM 没配好。
本库在 `Motor_Init()` 中拉高。若模块已硬件拉高，把 `BSP_MOTOR_HAS_STBY` 置 0。

**4. 方向脚同电平不会转**
`IN1 = IN2 = H` 是短路刹车，`IN1 = IN2 = L` 是高阻滑行。
电机转动要求两个方向脚**互补**且 PWM 为高。

**5. 换向前先把 PWM 清零**
直接在高占空比下翻转方向脚，会让 H 桥两臂瞬时同时导通。
TB6612 内部虽有死区时间，软件侧也不该抗跑。本库 `Motor_ApplySide()` 已处理。

**5b. 但那只解决电气时序，不解决机械换向冲击与启动冲击电流**
目标从 `+30cm/s` 直接改成 `-30cm/s`，或从 0 阶跃到 30cm/s，
电流尖峰与齿轮冲击照旧（双路同时阶跃可把 MCU 拉到欠压复位，
表现是「一给速度就重启」）。解决办法在上层加目标速度斜坡：
`common/motion_ramp.*`，原理与标定见 [`MOTION_RAMP.md`](MOTION_RAMP.md)。

---

## 编码器

**6. 16 位计数器会溢出，必须软件累计**
F103 的 TIM2/TIM3 是 16 位，本车1061 计数/圈，满量程只有约 62 圈，
跑几十秒就回绕。本库在 `encoder_tim.c` 用 Update 中断累计圈数并合成 int32_t。

**7. 读 CNT 与读溢出计数之间可能发生溢出**
先读溢出、再读 CNT、再复查溢出，不一致则重读。否则会拼出跳变几万的假计数，
表现为速度突然出现巨大尖峰并触发超速保护。本库 `Encoder_Compose()` 已处理。

**8. `TI1 and TI2` 才是 4 倍频**
选 `TI1 only` 只有 2 倍频，速度会差一半。选对了就**不要再在软件里乘3**。

**9. 方向反了不要改算法**
交换 A/B 接线，或改 `ENCODER_LEFT_DIRECTION_SIGN` 宏。

**10. 计数跳变不稳先加输入滤波**
CubeMX 里 IC Filter 试 5~10，并检查共地、屏蔽、开集电极编码器的上拉电际。

---

## PID

**11. 积分限幅太小会伪装成"Ki 不够"**
目标值长期达不到时，先查 `integralLimit` 是否帪住了，再考虑加 Ki。
本库自测用例 `selftest.c` 专门构造了这个对比场景。

**12. 改增益必须清积分**
旧积分是在旧 Ki 下累积的，换增益后会被瞬间放大。
本库 `PID_SetGains()` 内部自动清积分。

**13. 速度环慎用 D 项**
10ms 一拍的计数是整数，本车速度量化步长约 1.42 cm/s，
D 项会把量化噪声放大成输出抖动和电机噻叫。

**13b. 同一个根因的另一面：目标低于约 3 个量化步长时，速度环只能抖**
目标 5cm/s 只有约 3.5 个计数（量化误差 ±14%），2cm/s 只剩 1.4 个。
这是 M 法测速的**原理性**限制，不是调 PID 能解决的。
本库 `MotionRamp` 把低于 `minMoveCmps`（默认 4.3cm/s ≈ 3 倍量化步长）的
线速度目标直接判为停车；详见 [`MOTION_RAMP.md`](MOTION_RAMP.md)。

**14. 停车/切模式必须 Reset**
不清状态就重新使能，残留积分会让车窜出去。
本库 `SpeedControl_Enable(true)` 和 `App_SetMode()` 都会先 Reset。

---

## 串口

**15. Circular DMA 下不关半传输中断 → 每帧处理两次**
表现为"每条指令都执行两遇"。本库在 `UartLink_Init()` 和每次重启接收后
都调用 `__HAL_DMA_DISABLE_IT(hdmarx, DMA_IT_HT)`。

**16. 串口报错后不重启接收 → 串口"只能用一次"**
上位机在 STM32 复位期间持续发数据就会触发 ORE。
必须在 `HAL_UART_ErrorCallback` 里 Abort 后重新 `ReceiveToIdle_DMA`。
本库 `UartLink_OnError()` 已处理。

**17. 不要在中断里解析协议**
中断只把字节搬进环形缓冲，CRC 校验和业务逻辑放主循环。
否则长中断会破坏 10ms 控制节拍。

**18. 不要把缓冲区强转成结构体**
串口会粘包、拆包、丢字节，而且结构体受大小端与对齐影响，换编译器就崩。
用逐字节状态机（本库 `link_protocol.c`），噪声后能自动重新同步。

**19. CRC 错误帧绝不能刷新目标有效期**
否则干扰会让小车一直以为"视觉目标还有效"。本库直接丢弃并计数。

---

## OLED

**20. 整屏刷新约 25ms，绝不能放在控制中断里**
1024 字节 I2C 传输在 400kHz 下约 25ms。本库建议 100ms 刷一次。

**21. 显示失败不能影响控制**
I2C 没接好、地址不对（0x3C/0x3D）时，驱动必须静默失败而不是卡死等待。
本库所有 I2C 操作带超时并只累计错误计数，可用 `OLED_GetErrorCount()` 诊断。

**22. HAL 的 I2C 地址要左移一位**
`HAL_I2C_Master_Transmit` 需要 8 位地址，7 位地址 0x3C 要写成 `0x3C << 1`。

---

## 系统与工程

**23. 1ms 中断里只置标志位**
不做 PID、浮点除法、I2C。本库 `App_OnTick1ms()` 只递增计数和置标志。

**24. F103 没有 FPU**
Cortex-M3 浮点靠软件模拟。本库 10ms 内两路 PID 在 72MHz 下够用，
但不要在 1ms 中断里加浮点。F407 记得在工程里开启硬件 FPU。

**25. PA15 / PB3 / PB4 默认接 JTAG**
用这些脚做普通 IO 前要先关 JTAG 保留 SWD，否则读不到电平。
F1 上为 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`。

**26. CubeMX 重新生成会覆盖 USER CODE 之外的内容**
所以本库把 HAL 回调集中在 `examples/hal_callbacks.c`，
而不是散落在 `main.c` 的 USER CODE 区里每次手工补。

---

## ADC 多通道扫描（灰度探头模拟量版本）

**27. Continuous Conversion Mode 必须 Enable，否则只采一轮就停**
ADC 在 Single Conversion（默认）时，完成一次完整的规则组扫描后
`EOC` 置位并停机，DMA 不再收到新数据。
在 CubeMX 的 ADC1 Parameter Settings 里必须把
`Continuous Conversion Mode` 设为 **Enable**。
表现：上电后探头第一拍有数据，之后 `ADC_LineSensor_GetRaw()` 一直返回
同一组数值且 `s_dataReady` 不再刷新。

**28. DMA 数据宽度必须是 Half Word（16 位），不是 Byte（8 位）**
ADC1 输出是 12 位右对齐，存放在 16 位的 `ADC_DR` 寄存器里。
DMA 若配成 Byte，每次只搬低 8 位（最大 255），
ADC 高 4 位被丢弃，所有通道读出来上限都是 255，
归一化后大量通道永远在阈値附近抖动，表现为循迹不稳。
在 CubeMX DMA Settings 里，`Peripheral Data Width` 与
`Memory Data Width` 都必须设为 **Half Word**。

**28b. F103 没有"DMA Continuous Requests"选项，不用找**
该开关从 F4 开始才出现（对应 `ADC_CR2_DDS` 位）。
F103 只要把 DMA 模式设为 Circular，ADC 每完成一次转换就自动
向 DMA 发请求，不需要手动开关。
按 F4 教程在 CubeMX 里找不到该选项时直接跳过即可。

**29. 直接在主循环读 DMA 缓冲会读到半更新帧**
ADC Circular DMA 持续往 `s_dmaBuf` 写，主循环若直接读该数组，
有概率拿到"前 N 路是本次新值、后 M 路是上轮旧值"的混合帧。
归一化后出现奇怪的单通道跳变，表现和探头接触不良一模一样，很难排查。
正确做法：在 `HAL_ADC_ConvCpltCallback`（中断上下文）里把缓冲
快照到另一块 `s_snapBuf`，主循环只读快照——
`stm32/adc_line_sensor.c` 的双缓冲实现已处理此问题。
