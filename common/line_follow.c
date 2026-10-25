/**
 * @file line_follow.c
 * @brief 循迹外环实现。
 */
#include "line_follow.h"

#include "car_physical_config.h"

void LineFollow_LoadDefaultConfig(LineFollow_Config *config)
{
    if (config == 0) {
        return;
    }

    /*
     * 初始参数思路：
     * 权重跨度为 ±7，希望最大偏差时产生约 ±8cm/s 的左右速度差，
     * 因此 kp ≈ 8/7 ≈ 1.1。先把 kd 置 0 跑通，再逐步加 kd 压过弯超调。
     */
    config->kp = 1.1f;
    config->kd = 0.0f;
    config->baseSpeedCmps = CAR_RECOMMENDED_CRUISE_CMPS;
    config->minTargetCmps = 0.0f;
    config->maxTargetCmps = CAR_RECOMMENDED_MAX_CMPS;
    config->cornerSlowdownGain = 0.0f;
    config->lostLineStopCycles = 25U; /* 20ms × 25 = 500ms */
}

bool LineFollow_Init(LineFollow_State *s, const LineFollow_Config *config)
{
    if ((s == 0) || (config == 0)) {
        return false;
    }

    s->config = *config;
    LineFollow_Reset(s);
    return true;
}

void LineFollow_Reset(LineFollow_State *s)
{
    if (s == 0) {
        return;
    }

    s->previousError = 0.0f;
    s->correction = 0.0f;
    s->targetLeftCmps = 0.0f;
    s->targetRightCmps = 0.0f;
    s->lostLineCount = 0U;
    s->stopRequested = false;
}

void LineFollow_Update(LineFollow_State *s, const LineSensor_State *sensor)
{
    float error;
    float base;
    float absError;

    if ((s == 0) || (sensor == 0)) {
        return;
    }

    /* ---------- 异常分支 1：全黑。压上横线或终点标记，立即停。 ---------- */
    if (LineSensor_IsAllBlack(sensor)) {
        s->targetLeftCmps = 0.0f;
        s->targetRightCmps = 0.0f;
        s->correction = 0.0f;
        s->previousError = 0.0f;
        s->stopRequested = true;
        return;
    }

    /* ---------- 异常分支 2：全白丢线。沿用上次偏差继续找线。 ---------- */
    if (!LineSensor_IsLineFound(sensor)) {
        s->lostLineCount++;
        if ((s->config.lostLineStopCycles > 0U) &&
            (s->lostLineCount >= s->config.lostLineStopCycles)) {
            s->targetLeftCmps = 0.0f;
            s->targetRightCmps = 0.0f;
            s->correction = 0.0f;
            s->stopRequested = true;
            return;
        }
    } else {
        s->lostLineCount = 0U;
        s->stopRequested = false;
    }

    /* error 在丢线时已由 line_sensor 保持为 lastValidError。 */
    error = LineSensor_GetError(sensor);
    absError = (error < 0.0f) ? -error : error;

    /* ---------- PD 修正 ---------- */
    s->correction = s->config.kp * error +
                    s->config.kd * (error - s->previousError);
    s->previousError = error;

    /* ---------- 过弯减速：偏差越大基础速度越低 ---------- */
    base = s->config.baseSpeedCmps;
    if (s->config.cornerSlowdownGain > 0.0f) {
        base -= s->config.cornerSlowdownGain * absError;
        if (base < 0.0f) {
            base = 0.0f;
        }
    }

    /*
     * error > 0 表示线偏右，需要右转：左轮加速、右轮减速。
     * 若实车方向相反，不要改这里的符号，而是检查探头 bit0 是否真的在最左侧。
     */
    s->targetLeftCmps = ChessCar_ClampFloat(base + s->correction,
                                            s->config.minTargetCmps,
                                            s->config.maxTargetCmps);
    s->targetRightCmps = ChessCar_ClampFloat(base - s->correction,
                                             s->config.minTargetCmps,
                                             s->config.maxTargetCmps);
}

float LineFollow_GetTargetLeftCmps(const LineFollow_State *s)
{
    return (s != 0) ? s->targetLeftCmps : 0.0f;
}
float LineFollow_GetTargetRightCmps(const LineFollow_State *s)
{
    return (s != 0) ? s->targetRightCmps : 0.0f;
}
bool LineFollow_IsStopRequested(const LineFollow_State *s)
{
    return (s != 0) && s->stopRequested;
}
