/**
 * @file bsp_config.h
 * @brief STM32 侧的引脚、定时器与外设实例配置（唯一硬件映射入口）。
 *
 * 目标平台：STM32F103C8T6 / STM32F407（HAL 库 + STM32CubeMX 生成工程）
 *
 * 移植时只需要改这一个文件。所有 stm32 目录下的驱动都从这里取硬件句柄，
 * 不要在驱动 .c 里写死引脚号。
 *
 * ============== CubeMX 配置清单（照着勾） ==============
 *
 * 1) 电机 PWM —— TIM1 CH1/CH2
 *    - Clock Source: Internal Clock
 *    - Channel1: PWM Generation CH1    Channel2: PWM Generation CH2
 *    - Prescaler = 0, Counter Period (ARR) = 3599
 *      → 72MHz / (0+1) / (3599+1) = 20kHz，符合 TB6612FNG 的 100kHz 上限
 *    - 注意：多通道 PWM 必须对每个通道**分别**调用 HAL_TIM_PWM_Start，
 *      不能把通道宏按位或。
 *
 * 2) 左编码器 —— TIM2 (Combined Channels → Encoder Mode)
 *    右编码器 —— TIM3 (Combined Channels → Encoder Mode)
 *    - Encoder Mode: TI1 and TI2   ← 必须选这个才是 4 倍频
 *    - Counter Period (ARR) = 65535（用满 16 位，配合软件溢出累计）
 *    - IC1/IC2 Filter: 先填 0，若计数跳变不稳再试 5~10
 *    - 坑：方向反了不要改算法，交换 A/B 接线或改 bsp_config 里的符号
 *
 * 3) 控制节拍 —— TIM4 基础定时器，1ms 中断
 *    - Prescaler = 71, Counter Period = 999 → 72MHz/72/1000 = 1kHz
 *    - 使能 TIM4 global interrupt
 *    - 中断里只置标志位和递增计数，**绝不做 PID 运算或 I2C 刷屏**
 *
 * 4) 视觉/上位机串口 —— USART1，115200 8N1
 *    - DMA Settings: 添加 USART1_RX，Mode = Circular，优先级 Low
 *    - NVIC: 使能 USART1 global interrupt
 *    - 用 HAL_UARTEx_ReceiveToIdle_DMA + HAL_UARTEx_RxEventCallback
 *    - 坑：循环模式下需关掉 DMA 半传输中断，否则回调会被触发两次
 *
 * 5) OLED —— I2C1，400kHz Fast Mode
 *    - 7 位地址 0x3C（部分模块为 0x3D）
 *
 * 6) 灰度探头 —— ADC1 规则组 8 通道 + DMA（Circular）
 *    - Scan Conversion Mode: Enable，Continuous Conversion: Enable
 *    - 若用数字量 DO 版本探头，则改为 8 个 GPIO_Input，不需要 ADC
 */
#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/* ---- 选择芯片系列：按实际平台保留一行 ---- */
#define BSP_MCU_STM32F1
/* #define BSP_MCU_STM32F4 */

#if defined(BSP_MCU_STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(BSP_MCU_STM32F4)
#include "stm32f4xx_hal.h"
#else
#error "请在 bsp_config.h 中选择 BSP_MCU_STM32F1 或 BSP_MCU_STM32F4"
#endif

/* ===================== 电机 PWM ===================== */
#define BSP_MOTOR_PWM_TIM            htim1
#define BSP_MOTOR_PWM_CHANNEL_LEFT   TIM_CHANNEL_1
#define BSP_MOTOR_PWM_CHANNEL_RIGHT  TIM_CHANNEL_2

/* ===================== 电机方向 GPIO（TB6612FNG） ===================== */
/*
 * TB6612FNG 真值表要点：
 *   IN1=H IN2=L PWM=H  → CW   正转
 *   IN1=L IN2=H PWM=H  → CCW  反转
 *   IN1=H IN2=H        → Short brake  短路刹车
 *   IN1=L IN2=L        → Stop         高阻滑行
 *   STBY=L             → Standby 全部高阻，上电必须拉高
 */
#define BSP_MOTOR_LEFT_IN1_PORT      GPIOB
#define BSP_MOTOR_LEFT_IN1_PIN       GPIO_PIN_12
#define BSP_MOTOR_LEFT_IN2_PORT      GPIOB
#define BSP_MOTOR_LEFT_IN2_PIN       GPIO_PIN_13

#define BSP_MOTOR_RIGHT_IN1_PORT     GPIOB
#define BSP_MOTOR_RIGHT_IN1_PIN      GPIO_PIN_14
#define BSP_MOTOR_RIGHT_IN2_PORT     GPIOB
#define BSP_MOTOR_RIGHT_IN2_PIN      GPIO_PIN_15

/* STBY 引脚；若模块已硬件拉高，把 BSP_MOTOR_HAS_STBY 置 0 */
#define BSP_MOTOR_HAS_STBY           (1)
#define BSP_MOTOR_STBY_PORT          GPIOA
#define BSP_MOTOR_STBY_PIN           GPIO_PIN_11

/* ===================== 编码器定时器 ===================== */
#define BSP_ENCODER_LEFT_TIM         htim2
#define BSP_ENCODER_RIGHT_TIM        htim3
/* 16 位定时器用满量程，软件做溢出累计 */
#define BSP_ENCODER_TIM_PERIOD       (65536L)

/* ===================== 通信串口 ===================== */
#define BSP_LINK_UART                huart1
#define BSP_LINK_RX_DMA_BUFFER_SIZE  (128U)
#define BSP_LINK_RING_BUFFER_SIZE    (512U)

/* ===================== OLED ===================== */
#define BSP_OLED_I2C                 hi2c1
#define BSP_OLED_I2C_ADDR_7BIT       (0x3CU)
#define BSP_OLED_I2C_TIMEOUT_MS      (20U)

/* ===================== 灰度探头 ===================== */
/* 1 = 模拟量 AO + ADC；0 = 数字量 DO + GPIO */
#define BSP_LINE_SENSOR_USE_ADC      (1)
#define BSP_LINE_SENSOR_ADC          hadc1
#define BSP_LINE_SENSOR_CHANNELS     (8U)

/* 数字量版本的 8 路 DO 引脚（BSP_LINE_SENSOR_USE_ADC = 0 时生效） */
#define BSP_LINE_DO0_PORT  GPIOA
#define BSP_LINE_DO0_PIN   GPIO_PIN_4
#define BSP_LINE_DO1_PORT  GPIOA
#define BSP_LINE_DO1_PIN   GPIO_PIN_5
#define BSP_LINE_DO2_PORT  GPIOA
#define BSP_LINE_DO2_PIN   GPIO_PIN_8
#define BSP_LINE_DO3_PORT  GPIOA
#define BSP_LINE_DO3_PIN   GPIO_PIN_12
#define BSP_LINE_DO4_PORT  GPIOB
#define BSP_LINE_DO4_PIN   GPIO_PIN_0
#define BSP_LINE_DO5_PORT  GPIOB
#define BSP_LINE_DO5_PIN   GPIO_PIN_1
#define BSP_LINE_DO6_PORT  GPIOB
#define BSP_LINE_DO6_PIN   GPIO_PIN_5
#define BSP_LINE_DO7_PORT  GPIOB
#define BSP_LINE_DO7_PIN   GPIO_PIN_6

/* ---- CubeMX 生成的句柄声明 ---- */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
#if BSP_LINE_SENSOR_USE_ADC
extern ADC_HandleTypeDef hadc1;
#endif

#endif /* BSP_CONFIG_H */
