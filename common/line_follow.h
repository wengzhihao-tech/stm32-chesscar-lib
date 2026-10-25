/**
 * @file line_follow.h
 * @brief 循迹外环（串级 PID 的外层）：把横向偏差转成左右目标速度。
 *
 * 串级结构：
 *   循迹外环（20ms，PD）→ 目标左右速度 cm/s → 速度内环（10ms，PI）→ PWM
 *
 * 外环必须慢于内环，否则内环还没稳定外环就改目标，整车会左右摇摆。
 * 外环通常只用 Kp 与 Kd，Ki 保持 0：横向偏差的静差由车体几何决定，
 * 加积分只会让车在线上来回蠕动。
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "chesscar_common.h"
#include "line_sensor.h"
#include "pid_controller.h"

typedef struct {
    float kp;                 /**< 偏差比例增益，单位 (cm/s)/权重。 */
    float kd;                 /**< 偏差微分增益，抑制过弯超调。 */
    float baseSpeedCmps;      /**< 直道巡航速度。 */
    float minTargetCmps;      /**< 单侧目标速度下限，允许负值即允许内侧反转过急弯。 */
    float maxTargetCmps;      /**< 单侧目标速度上限。 */
    float cornerSlowdownGain; /**< 偏差大时的减速系数，0 表示过弯不减速。 */
    uint8_t lostLineStopCycles; /**< 连续丢线多少拍后停车，0 表示不自动停。 */
} LineFollow_Config;

typedef struct {
    LineFollow_Config config;
    float previousError;      /**< 上一个**控制周期**的偏差（不是采样周期！）。 */
    float correction;         /**< 本次转向修正量 cm/s。 */
    float targetLeftCmps;
    float targetRightCmps;
    uint8_t lostLineCount;
    bool stopRequested;       /**< 丢线超时或全黑，要求上层停车。 */
} LineFollow_State;

void LineFollow_LoadDefaultConfig(LineFollow_Config *config);
bool LineFollow_Init(LineFollow_State *s, const LineFollow_Config *config);
void LineFollow_Reset(LineFollow_State *s);

/**
 * @brief 执行一次外环计算，必须以 CAR_LINE_LOOP_PERIOD_MS 固定周期调用。
 *
 * @param sensor 已经更新过的探头状态。
 *
 * 重要：微分项使用的 previousError 必须是**上一个控制周期**的偏差。
 * 如果探头采样比控制周期快（例如 10ms 采样 / 20ms 控制），
 * 千万不要用采样周期的 lastError 覆盖它，否则微分项量纲错误、增益失效。
 */
void LineFollow_Update(LineFollow_State *s, const LineSensor_State *sensor);

float LineFollow_GetTargetLeftCmps(const LineFollow_State *s);
float LineFollow_GetTargetRightCmps(const LineFollow_State *s);
bool LineFollow_IsStopRequested(const LineFollow_State *s);

#endif /* LINE_FOLLOW_H */
