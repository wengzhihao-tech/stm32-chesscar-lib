/**
 * @file pid_controller.c
 * @brief 带积分限幅、条件抗饱和与输出限幅的离散 PID 实现。
 */
#include "pid_controller.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integralLimit, float outputLimit)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integralLimit = integralLimit;
    pid->outputLimit = outputLimit;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == 0) {
        return;
    }

    /* 保留 kp/ki/kd 与限幅参数，只清运行状态。 */
    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->previousError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->lastOutput = 0.0f;
}

void PID_SetGains(PID_Controller *pid, float kp, float ki, float kd)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    /* 旧积分是在旧 Ki 下累积的，换增益后必须清零，否则会瞬间放大。 */
    pid->integral = 0.0f;
}

static void PID_ShiftError(PID_Controller *pid, float target, float feedback)
{
    pid->target = target;
    pid->feedback = feedback;
    pid->previousError = pid->lastError;
    pid->lastError = pid->error;
    pid->error = pid->target - pid->feedback;
}

float PID_RunPosition(PID_Controller *pid, float target, float feedback)
{
    float candidateIntegral;
    float unsaturatedOutput;

    if (pid == 0) {
        return 0.0f;
    }

    PID_ShiftError(pid, target, feedback);

    candidateIntegral = pid->integral + pid->ki * pid->error;
    candidateIntegral = ChessCar_ClampFloat(candidateIntegral,
                                            -pid->integralLimit,
                                            pid->integralLimit);
    unsaturatedOutput = pid->kp * pid->error + candidateIntegral +
                        pid->kd * (pid->error - pid->lastError);

    /*
     * 条件积分抗饱和：
     * 当输出已经越过限幅、而误差仍在把它往同方向推时，本拍不接受新的积分；
     * 反向误差仍然允许积分回退，这样退出饱和时不会出现长时间迟滞。
     */
    if (((unsaturatedOutput > pid->outputLimit) && (pid->error > 0.0f)) ||
        ((unsaturatedOutput < -pid->outputLimit) && (pid->error < 0.0f))) {
        unsaturatedOutput = pid->kp * pid->error + pid->integral +
                            pid->kd * (pid->error - pid->lastError);
    } else {
        pid->integral = candidateIntegral;
    }

    pid->lastOutput = pid->output;
    pid->output = ChessCar_ClampFloat(unsaturatedOutput,
                                      -pid->outputLimit,
                                      pid->outputLimit);
    return pid->output;
}

float PID_RunIncremental(PID_Controller *pid, float target, float feedback)
{
    float delta;

    if (pid == 0) {
        return 0.0f;
    }

    PID_ShiftError(pid, target, feedback);

    delta = pid->kp * (pid->error - pid->lastError) +
            pid->ki * pid->error +
            pid->kd * (pid->error - 2.0f * pid->lastError + pid->previousError);

    pid->lastOutput = pid->output;
    pid->output = ChessCar_ClampFloat(pid->output + delta,
                                      -pid->outputLimit,
                                      pid->outputLimit);
    return pid->output;
}
