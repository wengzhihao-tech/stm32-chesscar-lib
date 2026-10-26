/**
 * @file pose_estimator.h
 * @brief 开放场地位姿估计：差速里程计 + IMU 航向 + 挡板绝对校正。
 *
 * 本赛题**没有引导线**，车必须自己知道身在何处，所以位姿估计是整车基础。
 *
 * 三级融合策略（由弱到强）：
 *   1. 差速里程计：编码器积分得到 (x, y, theta)。短时间很准，但会累积漂移，
 *      尤其是轮子打滑时误差无法恢复。
 *   2. IMU 航向替换：陀螺积分的航向角短期优于轮式里程计（不受打滑影响），
 *      用它替换里程计的 theta。但陀螺有零偏，长时间同样会漂。
 *   3. **挡板绝对校正（关键）**：场地四周有 80mm 高挡板，是唯一稳定的绝对基准。
 *      车贴近某面挡板时，用 ToF 侧向测距直接把该轴坐标"钉"回绝对值，
 *      一次校正就能清掉前面积累的全部漂移。
 *
 * 为什么必须有第 3 级：
 *   纯里程计在 3m×2m 场地里跑两三分钟，位置误差很容易到十几厘米，
 *   那时候"把棋子推进球门"就会变成"把棋子推到球门外面"。
 *
 * 本模块是平台无关的纯算法，不碰任何寄存器，可在 PC 上测试。
 */
#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include "chesscar_common.h"
#include "field_config.h"

/** 车辆位姿。单位：cm / deg。 */
typedef struct {
    float x;      /**< 场地坐标 X，0 ~ 300 cm。 */
    float y;      /**< 场地坐标 Y，0 ~ 200 cm。 */
    float theta;  /**< 航向角，deg，(-180, 180]，X 正方向为 0。 */
} Pose2D;

/** 哪一面挡板。用于贴边校正时指明参考基准。 */
typedef enum {
    WALL_NONE = 0,
    WALL_LEFT,    /**< X = 0 */
    WALL_RIGHT,   /**< X = FIELD_LENGTH_CM */
    WALL_BOTTOM,  /**< Y = 0 */
    WALL_TOP      /**< Y = FIELD_WIDTH_CM */
} Field_Wall;

typedef struct {
    float wheelTrackCm;       /**< 左右轮中心距，差速航向计算的分母，必须实测。 */
    bool useImuHeading;       /**< true = 用 IMU 航向替换里程计航向（推荐）。 */
    float imuHeadingWeight;   /**< IMU 航向融合权重 (0,1]，1 = 完全信任 IMU。 */
    /** ToF 传感器相对车体中心的安装偏移，用于把测距值换算到车体中心坐标。 */
    float tofFrontOffsetCm;
    float tofLeftOffsetCm;
    float tofRightOffsetCm;
    /** 贴边校正触发阈值：ToF 读数小于此值才认为"贴近挡板且可信"。 */
    float wallCorrectMaxRangeCm;
    /** 航向与挡板法线的最大夹角（deg），超过则侧向测距不能代表垂直距离。 */
    float wallCorrectMaxYawErrDeg;
} PoseEstimator_Config;

typedef struct {
    PoseEstimator_Config config;
    Pose2D pose;
    float lastLeftDistanceCm;   /**< 上一拍左轮累计里程。 */
    float lastRightDistanceCm;
    bool initialized;

    /* 诊断统计 */
    uint16_t wallCorrectionCount; /**< 成功贴边校正次数，0 说明校正逻辑从未生效。 */
    uint16_t rejectedCorrections; /**< 因姿态不满足而拒绝的校正次数。 */
    float totalPathCm;            /**< 累计行驶路程，用于评估漂移量级。 */
} PoseEstimator;

void PoseEstimator_LoadDefaultConfig(PoseEstimator_Config *config);

/** 初始化并设定发车位姿（发车区位置必须按抽签结果给出）。 */
bool PoseEstimator_Init(PoseEstimator *pe, const PoseEstimator_Config *config,
                        const Pose2D *startPose);

/** 强制重设位姿（例如重新发车）。 */
void PoseEstimator_SetPose(PoseEstimator *pe, const Pose2D *pose);

/**
 * @brief 用左右轮累计里程推进位姿。每个控制周期调用一次。
 *
 * @param leftDistanceCm  左轮**累计**里程（不是增量），取自 SpeedControl。
 * @param rightDistanceCm 右轮累计里程。
 * @param imuHeadingDeg   IMU 给出的绝对航向角（deg）。未使用 IMU 时传 NaN 或任意值，
 *                        并把 config.useImuHeading 置 false。
 */
void PoseEstimator_UpdateOdometry(PoseEstimator *pe,
                                  float leftDistanceCm,
                                  float rightDistanceCm,
                                  float imuHeadingDeg);

/**
 * @brief 用一次 ToF 侧向测距对指定挡板做绝对位置校正。
 *
 * @param wall      参考哪一面挡板。
 * @param rangeCm   ToF 测得的车体到挡板的距离（已含安装偏移补偿前的原始值）。
 * @param sensorOffsetCm 该传感器相对车体中心沿测量方向的偏移量。
 * @return true 表示校正已被接受。
 *
 * **会被拒绝的情况**（这是刻意设计，不是缺陷）：
 *   - 测距超出 wallCorrectMaxRangeCm：太远，可能测到的是对方车或棋子而不是挡板
 *   - 航向与挡板法线夹角过大：斜着测得到的是斜边距离，不等于垂直距离
 * 被拒绝时位姿保持不动，并累加 rejectedCorrections 供诊断。
 */
bool PoseEstimator_CorrectByWall(PoseEstimator *pe, Field_Wall wall,
                                 float rangeCm, float sensorOffsetCm);

/** 把航向角归一化到 (-180, 180]。 */
float Pose_NormalizeAngleDeg(float angleDeg);

/** 两点间直线距离 cm。 */
float Pose_DistanceCm(const Pose2D *a, const Pose2D *b);

/** 从 from 指向目标点 (tx,ty) 的方位角 deg。 */
float Pose_BearingToDeg(const Pose2D *from, float tx, float ty);

/** 目标点相对当前航向的转角偏差 deg，正值表示需左转。 */
float Pose_HeadingErrorDeg(const Pose2D *from, float tx, float ty);

/** 判断位姿是否在场地内（含安全边距）。越界说明定位已经失效或车要撞墙。 */
bool Pose_IsInsideField(const Pose2D *p, float marginCm);

const Pose2D *PoseEstimator_GetPose(const PoseEstimator *pe);

#endif /* POSE_ESTIMATOR_H */
