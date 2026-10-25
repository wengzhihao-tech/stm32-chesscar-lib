/**
 * @file app_chesscar.h
 * @brief 光电象棋小车主应用：把各模块按固定节拍串起来。
 *
 * 任务节拍设计（由 TIM4 的 1ms 中断驱动）：
 *   1ms   ：只递增计数器、置标志位（中断内绝不做运算）
 *   10ms  ：探头采样 + 速度内环 PID
 *   20ms  ：循迹外环 PD
 *   100ms ：OLED 刷新 + 状态帧上报
 *
 * 为什么外环必须慢于内环：
 *   内环还没把速度跟上来，外环就改目标，整车会左右摇摆。
 *   典型比例是外环周期取内环的 2~5 倍。
 */
#ifndef APP_CHESSCAR_H
#define APP_CHESSCAR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAR_MODE_IDLE = 0,     /**< 待机：电机断电，等待指令。 */
    CAR_MODE_LINE_FOLLOW,  /**< 循迹：外环 + 内环全开。 */
    CAR_MODE_MANUAL,       /**< 手动：只用速度内环，目标由串口给定。 */
    CAR_MODE_BENCH_TEST,   /**< 台架自检：开环扫 PWM，用于标定死区与前馈。 */
    CAR_MODE_FAULT         /**< 故障锁定：必须显式清故障才能退出。 */
} App_CarMode;

/** 上电初始化全部外设与控制器。在 HAL_Init 与 CubeMX 外设初始化之后调用。 */
void App_Init(void);

/** 主循环体，放在 while(1) 里反复调用，内部按标志位分发各节拍任务。 */
void App_Loop(void);

/** 在 TIM4 的 1ms 周期中断里调用（HAL_TIM_PeriodElapsedCallback）。 */
void App_OnTick1ms(void);

void App_SetMode(App_CarMode mode);
App_CarMode App_GetMode(void);

#endif /* APP_CHESSCAR_H */
