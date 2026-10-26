/**
 * @file motion_ramp.h
 * @brief 目标速度斜坡限幅（软启动 / 软停 / 过零换向）与低速可分辨下限保护。
 *
 * 平台无关：不含任何寄存器访问，可在 PC 上编译与单元测试。
 *
 * 为什么需要这一层（三个都是实打实会出问题的点）：
 *
 *  1. **启动冲击电流**。上层一次把目标从 0 跳到 40cm/s，速度环会立刻顶到
 *     满占空比。car_physical_config.h 记录的堵转电流是 2.0A，额定电流
 *     0.5A，两路电机同时阶跃很容易把电池拉垮导致 MCU 掉电重启。
 *     公开实现里普遍用「每个控制周期递增固定量」的软启动斜坡处理，
 *     例如 baihao774/stm32-tb6612-driver 的占空比每 20ms 递增 5%。
 *     <https://github.com/baihao774/stm32-tb6612-driver>
 *
 *  2. **换向穿通**。stm32/motor_tb6612.c 已做到「改方向前先清 PWM」，但那只
 *     解决了一拍之内的电平时序；如果上层把目标从 +30 直接改成 -30，车轮
 *     还在正转时 H 桥就被要求反转，机械冲击与电流尖峰依旧存在。本模块
 *     强制先减速过零、停一拍，再往反方向加速。
 *
 *  3. **低速不可分辨**。编码器量化步长 = Encoder_SpeedQuantumCmps()。
 *     当前配置（13PPR × 4 倍频 × 20.409 减速比，10ms 一拍）约 1.42cm/s，
 *     目标 5cm/s 只有约 3.5 个计数，量化误差 ±14%，速度环会持续抖动。
 *     本模块把「低于若干个量化步长」的目标直接判为停车，不让速度环在
 *     不可分辨区里挣扎。M 法测速在低速误差大是原理性的，不是调参能解决的：
 *     <https://mcu.eetrend.com/content/2025/100589256.html>
 *
 * 用法（放在上层与 speed_control 之间）：
 * @code
 *   MotionRamp_LoadDefaultConfig(&cfg);
 *   cfg.periodMs = CAR_SPEED_LOOP_PERIOD_MS;
 *   MotionRamp_Init(&ramp, &cfg);
 *   // 每拍：
 *   MotionRamp_SetTarget(&ramp, wantLinear, wantAngular);
 *   MotionRamp_Update(&ramp);
 *   SpeedControl_SetDifferential(&sc,
 *       MotionRamp_GetLinearCmps(&ramp), MotionRamp_GetAngularCmps(&ramp));
 * @endcode
 *
 * 紧急情况（丢线、通信超时、故障锁定）不要走斜坡：直接调
 * MotionRamp_EmergencyStop() 把状态清零，再调 SpeedControl_Stop()。
 */
#ifndef MOTION_RAMP_H
#define MOTION_RAMP_H

#include "chesscar_common.h"

typedef struct {
    /** 控制周期 ms，必须等于实际调用 MotionRamp_Update 的周期。 */
    float periodMs;

    /** 前进方向最大加速度 cm/s^2（目标增大时的限速）。 */
    float accelCmps2;
    /** 前进方向最大减速度 cm/s^2（目标减小时的限速），一般可比加速大。 */
    float decelCmps2;
    /** 角速度分量（左右轮速差）的最大变化率 cm/s^2，转向突变同样会打滑。 */
    float angularRateCmps2;

    /**
     * 低于此值的线速度目标直接判为 0，避免进入编码器不可分辨区。
     * 建议取 Encoder_SpeedQuantumCmps() 的 3 倍（当前配置约 4.3cm/s）。
     * 为 0 表示关闭该保护。
     */
    float minMoveCmps;

    /**
     * 换向时在零速处额外保持的拍数。TB6612 内部有死区时间，
     * 但齿轮箱与轮胎的机械反向冲击需要软件留出缓冲，建议 1~3 拍。
     */
    uint8_t reverseHoldCycles;
} MotionRamp_Config;

typedef struct {
    MotionRamp_Config config;

    float targetLinearCmps;  /**< 上层请求的线速度。 */
    float targetAngularCmps; /**< 上层请求的角速度分量。 */
    float linearCmps;        /**< 斜坡后的线速度输出。 */
    float angularCmps;       /**< 斜坡后的角速度输出。 */

    bool reversing;          /**< 正在执行「先过零再换向」的流程。 */
    uint8_t reverseHoldLeft; /**< 过零保持剩余拍数。 */
    bool limited;            /**< 本拍输出是否被斜坡/下限改写过。 */
} MotionRamp_State;

/** 载入一组对 MG310 + TB6612 偏保守的默认值（10ms 一拍）。 */
void MotionRamp_LoadDefaultConfig(MotionRamp_Config *config);

/** 初始化；periodMs/accel/decel 任一非正返回 false。 */
bool MotionRamp_Init(MotionRamp_State *s, const MotionRamp_Config *config);

/** 清零全部状态与目标（不输出速度）。 */
void MotionRamp_Reset(MotionRamp_State *s);

/** 设定上层期望的线速度与角速度分量，单位 cm/s。 */
void MotionRamp_SetTarget(MotionRamp_State *s, float linearCmps, float angularCmps);

/** 必须以固定周期调用一次，推进斜坡。 */
void MotionRamp_Update(MotionRamp_State *s);

/** 急停：目标与输出同时清零，不走斜坡。 */
void MotionRamp_EmergencyStop(MotionRamp_State *s);

float MotionRamp_GetLinearCmps(const MotionRamp_State *s);
float MotionRamp_GetAngularCmps(const MotionRamp_State *s);

/** 输出是否已经追上目标（可用于判断「加速完成，可以开始计里程」）。 */
bool MotionRamp_IsSettled(const MotionRamp_State *s);

/** 本拍是否处于换向过零流程中。 */
bool MotionRamp_IsReversing(const MotionRamp_State *s);

#endif /* MOTION_RAMP_H */
