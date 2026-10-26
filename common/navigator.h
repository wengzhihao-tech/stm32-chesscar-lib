/**
 * @file navigator.h
 * @brief 点到点导航：把"开到 (x,y)"转成左右轮目标速度，含挡板与对手规避。
 *
 * 这是本赛题替代循迹外环的模块。循迹赛里车只需要跟着线走，
 * 本赛题场上没有线，车必须自己朝一个坐标点开过去。
 *
 * 控制结构（仍然是串级，外环换成了导航）：
 *   导航外环（20ms）→ 目标左右速度 → 速度内环（10ms，PI）→ PWM
 *
 * 导航策略：
 *   - 航向偏差大时**先转再走**（原地转向），避免画大弧线浪费时间
 *   - 航向偏差小时边走边修，用航向误差做差速
 *   - 接近目标时按距离降速，防止冲过目标点
 *   - 推棋子时必须限制最大速度，推太快棋子会弹走而不是被推着走
 */
#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "chesscar_common.h"
#include "field_config.h"
#include "pose_estimator.h"

typedef struct {
    float headingKp;          /**< 航向误差 → 差速，单位 (cm/s)/deg。 */
    float headingKd;          /**< 航向误差微分，抑制摆头。 */
    float cruiseSpeedCmps;    /**< 空载巡航速度。 */
    float pushSpeedCmps;      /**< 推棋子时的速度，必须明显低于巡航速度。 */
    float approachSpeedCmps;  /**< 接近目标点时的低速。 */
    float spinSpeedCmps;      /**< 原地转向时的轮速。 */
    float spinThresholdDeg;   /**< 航向偏差超过此值则原地转向。 */
    float arriveToleranceCm;  /**< 距目标小于此值视为到达。 */
    float slowDownRadiusCm;   /**< 进入此半径开始按距离降速。 */
    float maxSpeedCmps;       /**< 单侧轮速上限。 */
} Navigator_Config;

typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_SPINNING,   /**< 原地转向对准目标。 */
    NAV_STATE_DRIVING,    /**< 朝目标行进。 */
    NAV_STATE_ARRIVED
} Navigator_State;

typedef struct {
    Navigator_Config config;
    Navigator_State state;
    float targetX;
    float targetY;
    bool pushMode;            /**< true = 正在推棋子，使用 pushSpeedCmps。 */
    float previousHeadingErr; /**< 上一控制周期的航向误差，供微分项使用。 */
    float targetLeftCmps;
    float targetRightCmps;
    float distanceToTargetCm;
    float headingErrorDeg;
    bool avoidingObstacle;    /**< 本拍是否因避障而改变了输出。 */
} Navigator;

void Navigator_LoadDefaultConfig(Navigator_Config *config);
bool Navigator_Init(Navigator *nav, const Navigator_Config *config);
void Navigator_Reset(Navigator *nav);

/** 设定目标点。pushMode 为 true 时使用推送速度上限。 */
void Navigator_SetTarget(Navigator *nav, float x, float y, bool pushMode);

/**
 * @brief 执行一次导航计算，须以固定周期（建议 20ms）调用。
 *
 * @param pose              当前位姿。
 * @param frontObstacleCm   正前方最近障碍距离，无障碍传一个大值（如 400）。
 * @param opponentDistCm    对方车辆距离，未检测到传大值。
 *
 * 避障优先级高于导航：检测到前方障碍或对方车辆过近时，
 * 本函数会主动减速或转向，**即使这会偏离目标点**。
 * 赛题明确规定恶意冲撞对方车辆要扣分或判负，所以这里宁可绕路。
 */
void Navigator_Update(Navigator *nav, const Pose2D *pose,
                      float frontObstacleCm, float opponentDistCm);

float Navigator_GetTargetLeftCmps(const Navigator *nav);
float Navigator_GetTargetRightCmps(const Navigator *nav);
bool Navigator_HasArrived(const Navigator *nav);
Navigator_State Navigator_GetState(const Navigator *nav);

#endif /* NAVIGATOR_H */
