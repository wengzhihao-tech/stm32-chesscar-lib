/**
 * @file pose_estimator.c
 * @brief 位姿估计实现。
 */
#include "pose_estimator.h"

#include <math.h>

#define POSE_DEG_TO_RAD (CHESSCAR_PI / 180.0f)
#define POSE_RAD_TO_DEG (180.0f / CHESSCAR_PI)

void PoseEstimator_LoadDefaultConfig(PoseEstimator_Config *config)
{
    if (config == 0) {
        return;
    }

    config->wheelTrackCm = 12.0f;   /* 必须按实车实测修改 */
    config->useImuHeading = true;
    config->imuHeadingWeight = 1.0f;

    config->tofFrontOffsetCm = 10.0f;
    config->tofLeftOffsetCm = 8.0f;
    config->tofRightOffsetCm = 8.0f;

    /*
     * 贴边校正阈值取 40cm：
     * 太大会把对方车辆、棋子堆误当成挡板；
     * 太小则很少有机会触发校正，漂移得不到修正。
     */
    config->wallCorrectMaxRangeCm = 40.0f;
    /*
     * 航向容差 12°：cos(12°) ≈ 0.978，即斜测带来的距离高估约 2.2%，
     * 在 40cm 量程下约 0.9cm，可以接受。再大就必须做 cos 补偿了。
     */
    config->wallCorrectMaxYawErrDeg = 12.0f;
}

float Pose_NormalizeAngleDeg(float angleDeg)
{
    /* 用 fmodf 一次收敛，避免大角度时 while 循环转很多圈。 */
    angleDeg = fmodf(angleDeg + 180.0f, 360.0f);
    if (angleDeg <= 0.0f) {
        /*
         * 用 <= 而不是 <，使结果区间为 (-180, 180] 而非 [-180, 180)。
         * 这一点很重要：车头朝 -X（正对左侧挡板）正好是 180°，
         * 若归一化成 -180°，贴边校正里的航向比对就会在边界上来回跳。
         */
        angleDeg += 360.0f;
    }
    return angleDeg - 180.0f;
}

bool PoseEstimator_Init(PoseEstimator *pe, const PoseEstimator_Config *config,
                        const Pose2D *startPose)
{
    if ((pe == 0) || (config == 0) || (config->wheelTrackCm <= 0.0f)) {
        return false;
    }

    pe->config = *config;
    if ((pe->config.imuHeadingWeight <= 0.0f) || (pe->config.imuHeadingWeight > 1.0f)) {
        pe->config.imuHeadingWeight = 1.0f;
    }

    if (startPose != 0) {
        pe->pose = *startPose;
    } else {
        pe->pose.x = 0.0f;
        pe->pose.y = 0.0f;
        pe->pose.theta = 0.0f;
    }
    pe->pose.theta = Pose_NormalizeAngleDeg(pe->pose.theta);

    pe->lastLeftDistanceCm = 0.0f;
    pe->lastRightDistanceCm = 0.0f;
    pe->initialized = false;
    pe->wallCorrectionCount = 0U;
    pe->rejectedCorrections = 0U;
    pe->totalPathCm = 0.0f;
    return true;
}

void PoseEstimator_SetPose(PoseEstimator *pe, const Pose2D *pose)
{
    if ((pe == 0) || (pose == 0)) {
        return;
    }
    pe->pose = *pose;
    pe->pose.theta = Pose_NormalizeAngleDeg(pe->pose.theta);
}

void PoseEstimator_UpdateOdometry(PoseEstimator *pe,
                                  float leftDistanceCm,
                                  float rightDistanceCm,
                                  float imuHeadingDeg)
{
    float dLeft, dRight, dCenter, dTheta;
    float midThetaRad;

    if (pe == 0) {
        return;
    }

    /* 首拍只记录基准，不做积分，避免把开机以来的累计里程当成一次位移。 */
    if (!pe->initialized) {
        pe->lastLeftDistanceCm = leftDistanceCm;
        pe->lastRightDistanceCm = rightDistanceCm;
        pe->initialized = true;
        return;
    }

    dLeft = leftDistanceCm - pe->lastLeftDistanceCm;
    dRight = rightDistanceCm - pe->lastRightDistanceCm;
    pe->lastLeftDistanceCm = leftDistanceCm;
    pe->lastRightDistanceCm = rightDistanceCm;

    dCenter = (dLeft + dRight) * 0.5f;
    pe->totalPathCm += CHESSCAR_ABS(dCenter);

    if (pe->config.useImuHeading) {
        /*
         * IMU 航向优先：轮子打滑时里程计航向会错得很厉害，
         * 而陀螺积分不受打滑影响。这是开放场地里最值得做的一次替换。
         */
        float imuNorm = Pose_NormalizeAngleDeg(imuHeadingDeg);
        float diff = Pose_NormalizeAngleDeg(imuNorm - pe->pose.theta);
        /* 用增量加权的方式融合，便于在 IMU 异常时降权而不是硬跳。 */
        dTheta = diff * pe->config.imuHeadingWeight;
    } else {
        /* 差速航向：两轮里程差除以轮距。轮距测不准会让转角系统性偏大或偏小。 */
        dTheta = (dRight - dLeft) / pe->config.wheelTrackCm * POSE_RAD_TO_DEG;
    }

    /*
     * 用「中点航向」积分位置，而不是用更新前或更新后的航向。
     * 这是二阶精度的做法：在一个周期内车是边转边走的，
     * 取中点能显著降低转弯时的位置误差。
     */
    midThetaRad = (pe->pose.theta + dTheta * 0.5f) * POSE_DEG_TO_RAD;
    pe->pose.x += dCenter * cosf(midThetaRad);
    pe->pose.y += dCenter * sinf(midThetaRad);
    pe->pose.theta = Pose_NormalizeAngleDeg(pe->pose.theta + dTheta);
}

/* 返回某面挡板的外法线方向（指向场地内部的反方向）对应的航向角。 */
static float Pose_WallNormalHeadingDeg(Field_Wall wall)
{
    switch (wall) {
        case WALL_LEFT:   return 180.0f; /* 车头朝 -X 才算正对左墙 */
        case WALL_RIGHT:  return 0.0f;
        case WALL_BOTTOM: return -90.0f;
        case WALL_TOP:    return 90.0f;
        default:          return 0.0f;
    }
}

bool PoseEstimator_CorrectByWall(PoseEstimator *pe, Field_Wall wall,
                                 float rangeCm, float sensorOffsetCm)
{
    float centerDistance;
    float yawErr;

    if ((pe == 0) || (wall == WALL_NONE)) {
        return false;
    }

    /* 超量程：很可能测到的是对方车辆或棋子堆，不是挡板，必须拒绝。 */
    if ((rangeCm <= 0.0f) || (rangeCm > pe->config.wallCorrectMaxRangeCm)) {
        pe->rejectedCorrections++;
        return false;
    }

    /*
     * 姿态检查：只有车大致正对挡板时，测距才等于垂直距离。
     * 斜着测到的是斜边，直接拿来当坐标会引入系统性误差。
     */
    yawErr = Pose_NormalizeAngleDeg(pe->pose.theta - Pose_WallNormalHeadingDeg(wall));
    if (CHESSCAR_ABS(yawErr) > pe->config.wallCorrectMaxYawErrDeg) {
        pe->rejectedCorrections++;
        return false;
    }

    /* 把传感器读数换算成"车体中心到挡板"的距离。 */
    centerDistance = rangeCm + sensorOffsetCm;

    /* 直接把对应轴的坐标钉回绝对值，一次清掉该轴的累积漂移。 */
    switch (wall) {
        case WALL_LEFT:   pe->pose.x = centerDistance; break;
        case WALL_RIGHT:  pe->pose.x = FIELD_LENGTH_CM - centerDistance; break;
        case WALL_BOTTOM: pe->pose.y = centerDistance; break;
        case WALL_TOP:    pe->pose.y = FIELD_WIDTH_CM - centerDistance; break;
        default: return false;
    }

    pe->wallCorrectionCount++;
    return true;
}

float Pose_DistanceCm(const Pose2D *a, const Pose2D *b)
{
    float dx, dy;

    if ((a == 0) || (b == 0)) {
        return 0.0f;
    }
    dx = b->x - a->x;
    dy = b->y - a->y;
    return sqrtf(dx * dx + dy * dy);
}

float Pose_BearingToDeg(const Pose2D *from, float tx, float ty)
{
    if (from == 0) {
        return 0.0f;
    }
    return atan2f(ty - from->y, tx - from->x) * POSE_RAD_TO_DEG;
}

float Pose_HeadingErrorDeg(const Pose2D *from, float tx, float ty)
{
    if (from == 0) {
        return 0.0f;
    }
    return Pose_NormalizeAngleDeg(Pose_BearingToDeg(from, tx, ty) - from->theta);
}

bool Pose_IsInsideField(const Pose2D *p, float marginCm)
{
    if (p == 0) {
        return false;
    }
    return (p->x >= marginCm) && (p->x <= (FIELD_LENGTH_CM - marginCm)) &&
           (p->y >= marginCm) && (p->y <= (FIELD_WIDTH_CM - marginCm));
}

const Pose2D *PoseEstimator_GetPose(const PoseEstimator *pe)
{
    return (pe != 0) ? &pe->pose : 0;
}
