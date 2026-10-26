/**
 * @file motion_ramp.c
 * @brief 目标速度斜坡限幅实现。
 */
#include "motion_ramp.h"

/* 单拍允许的最大变化量 = 加速度 × 周期(s)。 */
static float MotionRamp_StepLimit(float ratePerSec2, float periodMs)
{
    return ratePerSec2 * periodMs / 1000.0f;
}

/* 把 value 朝 target 移动，单拍最多走 step。 */
static float MotionRamp_Approach(float value, float target, float step, bool *changed)
{
    float diff = target - value;

    if (step <= 0.0f) {
        return target;
    }
    if (diff > step) {
        if (changed != 0) { *changed = true; }
        return value + step;
    }
    if (diff < -step) {
        if (changed != 0) { *changed = true; }
        return value - step;
    }
    /* 差值已小于单拍步长：直接落在目标上，不要在目标附近来回振荡。 */
    return target;
}

void MotionRamp_LoadDefaultConfig(MotionRamp_Config *config)
{
    if (config == 0) {
        return;
    }
    config->periodMs = 10.0f;
    /*
     * 60cm/s^2 ≈ 从静止加到 30cm/s 用 0.5s，对 3m×2m 的场地足够快，
     * 又不会把电池瞬间拉到堵转电流。减速给到 120cm/s^2：
     * 停得慢比停得猛更危险——越界和撞对手都是规则红线。
     */
    config->accelCmps2 = 60.0f;
    config->decelCmps2 = 120.0f;
    config->angularRateCmps2 = 90.0f;
    /*
     * 4.3cm/s ≈ 3 × 1.42cm/s（13PPR × 4 × 20.409、48mm 轮、10ms 一拍的量化步长）。
     * 换电机、换轮径或改控制周期后必须重算，不要照抄这个数。
     * 现场打印一次 Encoder_SpeedQuantumCmps() 就能得到正确值。
     */
    config->minMoveCmps = 4.3f;
    config->reverseHoldCycles = 2U;
}

bool MotionRamp_Init(MotionRamp_State *s, const MotionRamp_Config *config)
{
    if (s == 0 || config == 0) {
        return false;
    }
    if (config->periodMs <= 0.0f ||
        config->accelCmps2 <= 0.0f ||
        config->decelCmps2 <= 0.0f ||
        config->angularRateCmps2 <= 0.0f ||
        config->minMoveCmps < 0.0f) {
        return false;
    }

    s->config = *config;
    MotionRamp_Reset(s);
    return true;
}

void MotionRamp_Reset(MotionRamp_State *s)
{
    if (s == 0) {
        return;
    }
    s->targetLinearCmps = 0.0f;
    s->targetAngularCmps = 0.0f;
    s->linearCmps = 0.0f;
    s->angularCmps = 0.0f;
    s->reversing = false;
    s->reverseHoldLeft = 0U;
    s->limited = false;
}

void MotionRamp_SetTarget(MotionRamp_State *s, float linearCmps, float angularCmps)
{
    if (s == 0) {
        return;
    }

    /*
     * 低速保护只作用于线速度：原地转弯时线速度为 0 而角速度不为 0，
     * 如果连角速度也套用下限，车就没法做小角度微调对准了，
     * 而对准精度直接决定推棋子能不能进门。
     */
    if (s->config.minMoveCmps > 0.0f &&
        CHESSCAR_ABS(linearCmps) < s->config.minMoveCmps) {
        linearCmps = 0.0f;
    }

    s->targetLinearCmps = linearCmps;
    s->targetAngularCmps = angularCmps;
}

void MotionRamp_Update(MotionRamp_State *s)
{
    float accelStep;
    float decelStep;
    float angularStep;
    bool needReverse;

    if (s == 0) {
        return;
    }

    s->limited = false;
    accelStep = MotionRamp_StepLimit(s->config.accelCmps2, s->config.periodMs);
    decelStep = MotionRamp_StepLimit(s->config.decelCmps2, s->config.periodMs);
    angularStep = MotionRamp_StepLimit(s->config.angularRateCmps2, s->config.periodMs);

    /*
     * 换向判定：当前输出与目标异号且两者都不为 0。
     * 直接按斜坡穿过零点在数学上没问题，但物理上等价于「还在正转就命令反转」，
     * 所以这里显式分两段：先把线速度拉到 0 并保持若干拍，再放行反向加速。
     */
    needReverse = ((s->linearCmps > 0.0f && s->targetLinearCmps < 0.0f) ||
                   (s->linearCmps < 0.0f && s->targetLinearCmps > 0.0f));

    if (needReverse) {
        s->reversing = true;
        if (s->reverseHoldLeft == 0U) {
            s->reverseHoldLeft = s->config.reverseHoldCycles;
        }
    }

    if (s->reversing) {
        /* 过零阶段的中间目标是 0，用减速斜率（停得快一点更安全）。 */
        s->limited = true;
        s->linearCmps = MotionRamp_Approach(s->linearCmps, 0.0f, decelStep, 0);

        if (s->linearCmps == 0.0f) {
            if (s->reverseHoldLeft > 0U) {
                s->reverseHoldLeft--;
            }
            if (s->reverseHoldLeft == 0U) {
                /* 保持拍数用完，解除换向状态，下一拍开始反向加速。 */
                s->reversing = false;
            }
        }
        /*
         * 换向过程中角速度一律压到 0：此时车正在停下，
         * 保留转向分量只会让两轮一个停一个转，车身被拽偏，
         * 停下来时航向已经错了。
         */
        s->angularCmps = MotionRamp_Approach(s->angularCmps, 0.0f, angularStep, 0);
        return;
    }

    /*
     * 正常斜坡：按「远离零点」还是「靠近零点」选加速或减速斜率。
     * 注意要用绝对值比较：倒车时目标是负数，用带符号比较会选错斜率。
     */
    if (CHESSCAR_ABS(s->targetLinearCmps) > CHESSCAR_ABS(s->linearCmps)) {
        s->linearCmps = MotionRamp_Approach(s->linearCmps, s->targetLinearCmps,
                                            accelStep, &s->limited);
    } else {
        s->linearCmps = MotionRamp_Approach(s->linearCmps, s->targetLinearCmps,
                                            decelStep, &s->limited);
    }

    s->angularCmps = MotionRamp_Approach(s->angularCmps, s->targetAngularCmps,
                                         angularStep, &s->limited);
}

void MotionRamp_EmergencyStop(MotionRamp_State *s)
{
    if (s == 0) {
        return;
    }
    s->targetLinearCmps = 0.0f;
    s->targetAngularCmps = 0.0f;
    s->linearCmps = 0.0f;
    s->angularCmps = 0.0f;
    s->reversing = false;
    s->reverseHoldLeft = 0U;
    s->limited = true;
}

float MotionRamp_GetLinearCmps(const MotionRamp_State *s)
{
    return (s != 0) ? s->linearCmps : 0.0f;
}

float MotionRamp_GetAngularCmps(const MotionRamp_State *s)
{
    return (s != 0) ? s->angularCmps : 0.0f;
}

bool MotionRamp_IsSettled(const MotionRamp_State *s)
{
    if (s == 0) {
        return false;
    }
    return (!s->reversing) &&
           (s->linearCmps == s->targetLinearCmps) &&
           (s->angularCmps == s->targetAngularCmps);
}

bool MotionRamp_IsReversing(const MotionRamp_State *s)
{
    return (s != 0) ? s->reversing : false;
}
