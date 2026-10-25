/**
 * @file pid_controller.h
 * @brief 通用离散 PID 控制器（位置式 + 增量式），带积分限幅与条件抗饱和。
 *
 * 来源：从本机 smartcar 工程（MSPM0 平台，已上车验证）的 pid_controller 模块
 *       移植而来，算法层本身与芯片无关，可直接用于 STM32。
 *
 * 输入统一采用「目标值 target、反馈值 feedback」，内部误差为 target - feedback。
 * PID_Run* 必须在固定控制周期内调用（例如 TIM 基础定时器 10ms 中断或主循环节拍），
 * 函数内部不读取时间，所以 Ki/Kd 的量纲已经吸收了采样周期。
 *
 * 切换控制模式、修改目标值或在线改增益时必须调用 PID_Reset，
 * 否则历史积分会造成瞬时大输出。
 *
 * 参考资料：
 *  - 野火《电机应用开发实战指南—基于STM32》PID 算法的通俗解说
 *    https://doc.embedfire.com/motor/motor_tutorial/zh/latest/improve_part/PID_detailed.html
 *    要点：位置式 PID 需要积分限幅 + 输出限幅，增量式 PID 只需输出限幅。
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "chesscar_common.h"

typedef struct {
    float kp;             /**< 比例系数：偏差越大输出越大，响应越快，过大会振荡。 */
    float ki;             /**< 积分系数：消除长期静差，过大容易积分饱和与超调。 */
    float kd;             /**< 微分系数：抑制超调，对噪声敏感，需配合反馈滤波。 */

    float target;         /**< 当前目标值。 */
    float feedback;       /**< 当前反馈值。 */
    float error;          /**< 当前偏差 = target - feedback。 */
    float lastError;      /**< 上一拍偏差，用于一阶微分与增量式。 */
    float previousError;  /**< 上上拍偏差，用于增量式二阶差分 D 项。 */

    float integral;       /**< 积分累计量，仅位置式使用。 */
    float output;         /**< 当前输出控制量。 */
    float lastOutput;     /**< 上一拍输出，便于遥测与限速判断。 */

    float integralLimit;  /**< 积分限幅，防止积分饱和。 */
    float outputLimit;    /**< 输出限幅，通常对应最大 PWM，软件最后一道保护。 */
} PID_Controller;

/** 初始化增益与限幅，并清空运行状态。 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integralLimit, float outputLimit);

/** 只清空运行状态，保留 kp/ki/kd 与限幅；切换模式或重新起步前必须调用。 */
void PID_Reset(PID_Controller *pid);

/** 在线修改增益（会自动清积分，避免旧积分被新 Ki 放大）。 */
void PID_SetGains(PID_Controller *pid, float kp, float ki, float kd);

/**
 * @brief 位置式 PID：输出直接代表本次控制量。
 *
 * output = Kp*err + integral + Kd*(err - lastErr)
 * 适合距离环、角度环、转向环与一般速度环。
 * 内部采用「条件积分抗饱和」：误差继续把输出推向饱和时暂停累加，
 * 反向误差仍允许积分回退，这是比单纯积分限幅更稳的做法。
 */
float PID_RunPosition(PID_Controller *pid, float target, float feedback);

/**
 * @brief 增量式 PID：本次只算输出变化量再叠加到上一次输出。
 *
 * delta = Kp*(err - lastErr) + Ki*err + Kd*(err - 2*lastErr + prevErr)
 * 输出变化平滑，适合电机速度闭环；手自动切换冲击小，但存在积分截断效应。
 */
float PID_RunIncremental(PID_Controller *pid, float target, float feedback);

#endif /* PID_CONTROLLER_H */
