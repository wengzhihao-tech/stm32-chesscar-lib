/**
 * @file navigator.c
 * @brief 点到点导航实现。
 */
#include "navigator.h"

#include <math.h>

void Navigator_LoadDefaultConfig(Navigator_Config *config)
{
    if (config == 0) {
        return;
    }

    /*
     * headingKp = 0.35 (cm/s)/deg：航向偏差 20° 时产生约 7cm/s 的差速，
     * 在 30cm/s 巡航下对应不太激烈的转弯。先用小值跑通再加大。
     */
    config->headingKp = 0.35f;
    config->headingKd = 0.05f;

    config->cruiseSpeedCmps = 30.0f;
    /*
     * 推送速度必须明显低于巡航：棋子只有约 9mm 厚、重量很轻，
     * 推太快会把棋子弹开或翻倒，而不是稳定推着它走。
     */
    config->pushSpeedCmps = 14.0f;
    config->approachSpeedCmps = 12.0f;
    config->spinSpeedCmps = 10.0f;

    /*
     * 航向偏差超过 35° 就原地转向。
     * 阈值太小会频繁停下转向，太大则会画出很大的弧线浪费时间。
     */
    config->spinThresholdDeg = 35.0f;
    config->arriveToleranceCm = 3.0f;
    config->slowDownRadiusCm = 20.0f;
    config->maxSpeedCmps = 45.0f;
}

bool Navigator_Init(Navigator *nav, const Navigator_Config *config)
{
    if ((nav == 0) || (config == 0)) {
        return false;
    }
    nav->config = *config;
    Navigator_Reset(nav);
    return true;
}

void Navigator_Reset(Navigator *nav)
{
    if (nav == 0) {
        return;
    }
    nav->state = NAV_STATE_IDLE;
    nav->targetX = 0.0f;
    nav->targetY = 0.0f;
    nav->pushMode = false;
    nav->previousHeadingErr = 0.0f;
    nav->targetLeftCmps = 0.0f;
    nav->targetRightCmps = 0.0f;
    nav->distanceToTargetCm = 0.0f;
    nav->headingErrorDeg = 0.0f;
    nav->avoidingObstacle = false;
}

void Navigator_SetTarget(Navigator *nav, float x, float y, bool pushMode)
{
    bool targetChanged;

    if (nav == 0) {
        return;
    }

    /*
     * 关键：上层状态机会**每拍都调用本函数**（因为棋子位置会随视觉更新而变），
     * 所以这里必须判断目标是否真的变了。
     * 若无条件把状态重置为 SPINNING，导航就永远停在"转向中"，
     * 再也不会报告"已到达"，上层状态机会卡死在接近阶段直到超时。
     *
     * 阈值取 1cm：小于视觉定位误差，既能吸收抖动，
     * 又能在目标真正移动时及时重新对准。
     */
    targetChanged = (CHESSCAR_ABS(x - nav->targetX) > 1.0f) ||
                    (CHESSCAR_ABS(y - nav->targetY) > 1.0f) ||
                    (pushMode != nav->pushMode);

    nav->targetX = x;
    nav->targetY = y;
    nav->pushMode = pushMode;

    if (targetChanged || (nav->state == NAV_STATE_IDLE)) {
        nav->state = NAV_STATE_SPINNING; /* 新目标先对准，避免一上来就画弧 */
        nav->previousHeadingErr = 0.0f;
    }
}

void Navigator_Update(Navigator *nav, const Pose2D *pose,
                      float frontObstacleCm, float opponentDistCm)
{
    float headingErr;
    float distance;
    float baseSpeed;
    float correction;

    if ((nav == 0) || (pose == 0)) {
        return;
    }

    nav->avoidingObstacle = false;

    distance = sqrtf((nav->targetX - pose->x) * (nav->targetX - pose->x) +
                     (nav->targetY - pose->y) * (nav->targetY - pose->y));
    headingErr = Pose_HeadingErrorDeg(pose, nav->targetX, nav->targetY);
    nav->distanceToTargetCm = distance;
    nav->headingErrorDeg = headingErr;

    /* ---------- 到达判定 ---------- */
    if ((nav->state != NAV_STATE_IDLE) && (distance <= nav->config.arriveToleranceCm)) {
        nav->state = NAV_STATE_ARRIVED;
        nav->targetLeftCmps = 0.0f;
        nav->targetRightCmps = 0.0f;
        return;
    }

    if ((nav->state == NAV_STATE_IDLE) || (nav->state == NAV_STATE_ARRIVED)) {
        nav->targetLeftCmps = 0.0f;
        nav->targetRightCmps = 0.0f;
        return;
    }

    /*
     * ---------- 避障优先：规则高于效率 ----------
     * 赛题规定恶意冲撞对方车辆会扣分或直接判负，
     * 所以只要对方车太近，立刻停下并原地转开，哪怕放弃当前目标。
     */
    if (opponentDistCm < OPPONENT_AVOID_DISTANCE_CM) {
        nav->avoidingObstacle = true;
        /* 原地右转让开，不前进，避免任何接触。 */
        nav->targetLeftCmps = nav->config.spinSpeedCmps;
        nav->targetRightCmps = -nav->config.spinSpeedCmps;
        return;
    }

    /*
     * 前方有障碍（挡板或棋子堆）：
     * 推棋子时前方本来就"有东西"（就是被推的棋子），
     * 所以推送模式下不把近距离障碍当成需要躲避的对象。
     */
    if (!nav->pushMode && (frontObstacleCm < 15.0f)) {
        nav->avoidingObstacle = true;
        nav->targetLeftCmps = nav->config.spinSpeedCmps;
        nav->targetRightCmps = -nav->config.spinSpeedCmps;
        return;
    }

    /* ---------- 原地转向：偏差大时先对准 ---------- */
    if (CHESSCAR_ABS(headingErr) > nav->config.spinThresholdDeg) {
        nav->state = NAV_STATE_SPINNING;
        /* headingErr > 0 需左转：左轮后退、右轮前进。 */
        if (headingErr > 0.0f) {
            nav->targetLeftCmps = -nav->config.spinSpeedCmps;
            nav->targetRightCmps = nav->config.spinSpeedCmps;
        } else {
            nav->targetLeftCmps = nav->config.spinSpeedCmps;
            nav->targetRightCmps = -nav->config.spinSpeedCmps;
        }
        nav->previousHeadingErr = headingErr;
        return;
    }

    /* ---------- 行进：边走边修航向 ---------- */
    nav->state = NAV_STATE_DRIVING;

    baseSpeed = nav->pushMode ? nav->config.pushSpeedCmps
                              : nav->config.cruiseSpeedCmps;

    /* 接近目标时按距离线性降速，避免冲过目标点来回震荡。 */
    if (distance < nav->config.slowDownRadiusCm) {
        float scale = distance / nav->config.slowDownRadiusCm;
        float minSpeed = nav->config.approachSpeedCmps;
        baseSpeed = minSpeed + (baseSpeed - minSpeed) * scale;
    }

    correction = nav->config.headingKp * headingErr +
                 nav->config.headingKd * (headingErr - nav->previousHeadingErr);
    nav->previousHeadingErr = headingErr;

    /* headingErr > 0 需左转 → 右轮快、左轮慢。 */
    nav->targetLeftCmps = ChessCar_ClampFloat(baseSpeed - correction,
                                              -nav->config.maxSpeedCmps,
                                              nav->config.maxSpeedCmps);
    nav->targetRightCmps = ChessCar_ClampFloat(baseSpeed + correction,
                                               -nav->config.maxSpeedCmps,
                                               nav->config.maxSpeedCmps);
}

float Navigator_GetTargetLeftCmps(const Navigator *nav)
{
    return (nav != 0) ? nav->targetLeftCmps : 0.0f;
}
float Navigator_GetTargetRightCmps(const Navigator *nav)
{
    return (nav != 0) ? nav->targetRightCmps : 0.0f;
}
bool Navigator_HasArrived(const Navigator *nav)
{
    return (nav != 0) && (nav->state == NAV_STATE_ARRIVED);
}
Navigator_State Navigator_GetState(const Navigator *nav)
{
    return (nav != 0) ? nav->state : NAV_STATE_IDLE;
}
