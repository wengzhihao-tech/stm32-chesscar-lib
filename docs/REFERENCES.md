# 公开参考资料索引

本次调研收集的可参考实现与技术文章，按模块分类。链接均为公开可访问页面。

---

## TB6612FNG 电机驱动

- **TB6612FNG 官方数据手册**（真值表、fPWM 上限 100kHz、VM 2.5~13.5V、
  每通道 1.2A 平均 / 3.2A 峰值、内置热关断与死区时间）
  <https://www.tme.eu/Document/f69c44f5b284761d41a1fa952256eb8d/TB6612FNG.pdf>
  <https://tams.informatik.uni-hamburg.de/lehre/2019ss/vorlesung/es/doc/tb6612fng.pdf>
- **TB6612FNG 引脚与接线说明**（STBY 必须拉高、方向脚同电平不转、A/B 并联驱动大电流电机）
  <https://www.tinkered.ai/components/tb6612fng-motor-driver>
  <https://www.sparkfun.com/sparkfun-motor-driver-dual-tb6612fng-1a.html>
- **PWM 与 TB6612FNG 驱动电机（两轮自平衡小车实战指南）**
  含完整 CubeMX 配置步骤、24kHz PWM 的 PSC/ARR 取值、占空比公式
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

- **野火 · PID 算法的通俗解说**（离散化推导、位置式需积分限幅+输出限幅、
  增量式只需输出限幅、两者适用对象对比）
  <https://doc.embedfire.com/motor/motor_tutorial/zh/latest/improve_part/PID_detailed.html>
  <https://github.com/Embedfire-motor/ebf_motor_tutorial_stm32f407/blob/master/improve_part/PID_detailed.rst>
- **野火 · 直流电机速度环控制实现**（位置式与增量式两套例程、定时中断执行 PID 的编程要点）
  <https://doc.embedfire.com/motor/motor_tutorial/zh/latest/improve_part/dc_motor_speed_loop_control.html>
- **野火 · 直流电机位置环控制实现**（增量式公式、误差死区处理）
  <https://doc.embedfire.com/mcu/renesas/motor/zh/latest/doc/improve_part/dc_motor_pos_loop_control.html>
- **野火 · 位置速度双环控制实现**（外环输出作为内环输入、目标速度上限钳位）
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
  两个中断向量须设相同抢占优先级）
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
