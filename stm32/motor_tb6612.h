/**
 * @file motor_tb6612.h
 * @brief TB6612FNG 双路直流电机驱动（STM32 HAL：TIM PWM + 方向 GPIO）。
 *
 * 约定：**正值统一表示车辆前进**，与左右电机的物理安装方向无关。
 * 若某侧给正 PWM 却后退，改 bsp_config.h 里对应的 IN1/IN2 顺序，
 * 或在 speed_control 的 leftMotorSign/rightMotorSign 里翻转，
 * 不要在上层算法里加负号——那会让后续所有人都看不懂。
 *
 * 禁止绕过本层直接写 TIMx->CCRx，也不要在中断里调用本层的刹车函数。
 */
#ifndef MOTOR_TB6612_H
#define MOTOR_TB6612_H

#include "bsp_config.h"
#include <stdint.h>

/** 初始化：启动两路 PWM、拉高 STBY、输出零速。 */
void Motor_Init(void);

/** 同时设置左右电机，范围 [-CAR_PWM_OUTPUT_LIMIT, +CAR_PWM_OUTPUT_LIMIT]。 */
void Motor_SetSides(int16_t left, int16_t right);

void Motor_SetLeft(int16_t speed);
void Motor_SetRight(int16_t speed);

/** 停止：IN1=IN2=L 高阻滑行，电机自由减速。 */
void Motor_Stop(void);

/** 刹车：IN1=IN2=H 短路刹车，制动力强，频繁使用会增加驱动热耗。 */
void Motor_Brake(void);

/** 进入待机（STBY=L），两路 H 桥全部高阻。 */
void Motor_Standby(void);

/** 读回最后施加的方向：1 前进，-1 后退，0 从未转动。 */
int8_t Motor_GetLeftDirection(void);
int8_t Motor_GetRightDirection(void);

#endif /* MOTOR_TB6612_H */
