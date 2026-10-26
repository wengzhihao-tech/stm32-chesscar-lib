/**
 * @file match_fsm.c
 * @brief 比赛流程状态机实现。
 */
#include "match_fsm.h"

#include <math.h>

/*
 * 搜索巡视点：覆盖场地主要区域。
 * 坐标以"我方球门在 X=0"（甲队）为基准定义，
 * 乙队通过 X 轴镜像得到，这样一套巡视点两个阵营都能用。
 *
 * 第一个点是棋子堆放区——赛题说棋子随机投放，但中央堆放区密度最高，
 * 优先去那里期望收益最大。
 */
typedef struct { float x, y; } Match_Waypoint;

static const Match_Waypoint MATCH_WAYPOINTS[] = {
    { 150.0f, 100.0f }, /* 中央棋子堆放区 */
    {  80.0f,  60.0f },
    {  80.0f, 140.0f },
    { 150.0f,  45.0f },
    { 150.0f, 155.0f },
    { 220.0f,  70.0f },
    { 220.0f, 130.0f }
};
#define MATCH_WAYPOINT_COUNT \
    (sizeof(MATCH_WAYPOINTS) / sizeof(MATCH_WAYPOINTS[0]))

void Match_LoadDefaultConfig(Match_Config *config)
{
    if (config == 0) {
        return;
    }

    /*
     * ⚠ 比赛总时长赛题未给出（只提到平局时有 60 秒金球加赛）。
     * 这里按常见的 3 分钟占位，**必须按现场规则手册修改**。
     */
    config->matchDurationMs = 180000U;

    config->searchTimeoutMs = 4000U;
    config->approachTimeoutMs = 8000U;
    config->alignTimeoutMs = 3000U;
    config->pushTimeoutMs = 12000U;

    /*
     * 推送起始点距棋子 15cm：
     * 要大于车体前端到旋转中心的距离，否则接近过程中车就先把棋子碰开了。
     * 装了推板后要按推板前伸量重新调整。
     */
    config->approachStandoffCm = 15.0f;
    config->alignToleranceDeg = 8.0f;
    /*
     * 推丢判定 8cm：棋子直径约 3.2cm，推板宽度有限，
     * 偏离轴线超过 8cm 基本已经脱离推板覆盖范围。
     */
    config->pushLostDistanceCm = 8.0f;

    config->pushDistanceWeight = 2.0f;
    config->scoreWeight = 10.0f;
    config->retreatDistanceCm = 30.0f;
}

bool Match_Init(Match_Fsm *fsm, const Match_Config *config)
{
    if ((fsm == 0) || (config == 0)) {
        return false;
    }

    fsm->config = *config;
    fsm->state = MATCH_STATE_INIT;
    fsm->stateEnterMs = 0U;
    fsm->matchStartMs = 0U;
    fsm->currentTargetIndex = -1;
    fsm->approachX = 0.0f;
    fsm->approachY = 0.0f;
    fsm->searchWaypointIndex = 0U;
    fsm->pushAttempts = 0U;
    fsm->pushSuccesses = 0U;
    fsm->pushLostCount = 0U;
    fsm->searchTimeouts = 0U;
    fsm->recoverCount = 0U;
    return true;
}

static void Match_EnterState(Match_Fsm *fsm, Match_State state, uint32_t nowMs)
{
    fsm->state = state;
    fsm->stateEnterMs = nowMs;
}

static uint32_t Match_ElapsedInState(const Match_Fsm *fsm, uint32_t nowMs)
{
    return (uint32_t)(nowMs - fsm->stateEnterMs); /* 减法保证回绕安全 */
}

void Match_Start(Match_Fsm *fsm, uint32_t nowMs)
{
    if (fsm == 0) {
        return;
    }
    fsm->matchStartMs = nowMs;
    fsm->currentTargetIndex = -1;
    Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
}

void Match_GetSearchWaypoint(const Match_Fsm *fsm, uint8_t ourSide,
                             float *outX, float *outY)
{
    uint8_t idx;

    if ((fsm == 0) || (outX == 0) || (outY == 0)) {
        return;
    }

    idx = (uint8_t)(fsm->searchWaypointIndex % MATCH_WAYPOINT_COUNT);
    *outY = MATCH_WAYPOINTS[idx].y;

    /* 乙队把 X 镜像，使"靠近我方球门"的语义对两个阵营一致。 */
    if (ourSide == FIELD_SIDE_BLUE) {
        *outX = MATCH_WAYPOINTS[idx].x;
    } else {
        *outX = FIELD_LENGTH_CM - MATCH_WAYPOINTS[idx].x;
    }
}

/* 计算棋子中心到「球门 → 推送起点」轴线的偏移，用于判断是否推丢。 */
static float Match_PushLateralOffsetCm(const Chess_TargetManager *mgr,
                                       const Chess_Piece *piece,
                                       const Pose2D *carPose)
{
    float axX, axY, axLen;
    float relX, relY;

    /* 推送方向：从车指向球门。 */
    axX = mgr->goalX - carPose->x;
    axY = mgr->goalY - carPose->y;
    axLen = sqrtf(axX * axX + axY * axY);
    if (axLen < 1e-3f) {
        return 0.0f;
    }
    axX /= axLen;
    axY /= axLen;

    relX = piece->x - carPose->x;
    relY = piece->y - carPose->y;

    /* 叉乘的绝对值 = 点到轴线的垂直距离。 */
    return CHESSCAR_ABS(relX * axY - relY * axX);
}

void Match_Update(Match_Fsm *fsm, uint32_t nowMs,
                  Chess_TargetManager *mgr,
                  const PoseEstimator *pe,
                  Navigator *nav,
                  float opponentDistCm)
{
    const Pose2D *pose;
    const Chess_Piece *target;
    float wx, wy;

    if ((fsm == 0) || (mgr == 0) || (pe == 0) || (nav == 0)) {
        return;
    }

    pose = PoseEstimator_GetPose(pe);
    if (pose == 0) {
        return;
    }

    /* ---------- 全局：比赛时间到即停 ---------- */
    if ((fsm->state != MATCH_STATE_INIT) && (fsm->state != MATCH_STATE_FINISHED)) {
        if ((uint32_t)(nowMs - fsm->matchStartMs) >= fsm->config.matchDurationMs) {
            Match_EnterState(fsm, MATCH_STATE_FINISHED, nowMs);
            Navigator_Reset(nav);
            return;
        }
    }

    /* ---------- 全局：位姿越界说明定位已失效 ---------- */
    if ((fsm->state == MATCH_STATE_APPROACH) || (fsm->state == MATCH_STATE_PUSH)) {
        if (!Pose_IsInsideField(pose, 2.0f)) {
            /*
             * 位姿跑到场地外是不可能的物理状态，只能是里程计漂移太大。
             * 继续按错误位姿行动会撞墙，所以进入恢复流程。
             */
            fsm->recoverCount++;
            fsm->currentTargetIndex = -1;
            Match_EnterState(fsm, MATCH_STATE_RECOVER, nowMs);
        }
    }

    switch (fsm->state) {
        case MATCH_STATE_INIT:
            /* 等待 Match_Start 被调用。保持静止。 */
            Navigator_SetTarget(nav, pose->x, pose->y, false);
            break;

        case MATCH_STATE_SEARCH: {
            int8_t best = Chess_SelectBestTarget(mgr, pose,
                                                 fsm->config.pushDistanceWeight,
                                                 fsm->config.scoreWeight);
            if (best >= 0) {
                fsm->currentTargetIndex = best;
                Chess_ComputeApproachPoint(mgr, (uint8_t)best,
                                           fsm->config.approachStandoffCm,
                                           &fsm->approachX, &fsm->approachY);
                Navigator_SetTarget(nav, fsm->approachX, fsm->approachY, false);
                Match_EnterState(fsm, MATCH_STATE_APPROACH, nowMs);
                break;
            }

            /* 没找到目标：沿巡视点移动，边走边让视觉扫描。 */
            Match_GetSearchWaypoint(fsm, mgr->ourSide, &wx, &wy);
            Navigator_SetTarget(nav, wx, wy, false);

            if (Navigator_HasArrived(nav) ||
                (Match_ElapsedInState(fsm, nowMs) > fsm->config.searchTimeoutMs)) {
                /* 换下一个巡视点，避免停在原地空转浪费时间。 */
                fsm->searchWaypointIndex++;
                fsm->searchTimeouts++;
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
            }
            break;
        }

        case MATCH_STATE_APPROACH:
            target = Chess_GetPiece(mgr, (uint8_t)fsm->currentTargetIndex);
            if ((fsm->currentTargetIndex < 0) || (target == 0) ||
                !target->valid || target->scored) {
                /* 目标消失（被对方推走或原本是误检），重新搜索。 */
                fsm->currentTargetIndex = -1;
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
                break;
            }

            /* 棋子可能移动了，持续更新推送起始点。 */
            Chess_ComputeApproachPoint(mgr, (uint8_t)fsm->currentTargetIndex,
                                       fsm->config.approachStandoffCm,
                                       &fsm->approachX, &fsm->approachY);
            Navigator_SetTarget(nav, fsm->approachX, fsm->approachY, false);

            if (Navigator_HasArrived(nav)) {
                Match_EnterState(fsm, MATCH_STATE_ALIGN, nowMs);
            } else if (Match_ElapsedInState(fsm, nowMs) > fsm->config.approachTimeoutMs) {
                /* 接近超时：可能被对方挡住，放弃这枚换一个目标。 */
                fsm->currentTargetIndex = -1;
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
            }
            break;

        case MATCH_STATE_ALIGN: {
            /*
             * 对准球门再推，是推送成功率的关键。
             * 没对准就推，棋子会被斜着推出轴线，很快脱离推板。
             */
            float headingErr = Pose_HeadingErrorDeg(pose, mgr->goalX, mgr->goalY);

            Navigator_SetTarget(nav, mgr->goalX, mgr->goalY, true);

            if (CHESSCAR_ABS(headingErr) <= fsm->config.alignToleranceDeg) {
                fsm->pushAttempts++;
                Match_EnterState(fsm, MATCH_STATE_PUSH, nowMs);
            } else if (Match_ElapsedInState(fsm, nowMs) > fsm->config.alignTimeoutMs) {
                fsm->currentTargetIndex = -1;
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
            }
            break;
        }

        case MATCH_STATE_PUSH:
            target = Chess_GetPiece(mgr, (uint8_t)fsm->currentTargetIndex);
            if ((fsm->currentTargetIndex < 0) || (target == 0) || !target->valid) {
                fsm->currentTargetIndex = -1;
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
                break;
            }

            /* 低速朝球门推。pushMode = true 让导航使用推送速度。 */
            Navigator_SetTarget(nav, mgr->goalX, mgr->goalY, true);

            /* 进门判定：棋子整体进入球门区。 */
            if (Chess_IsInOurGoal(mgr, (uint8_t)fsm->currentTargetIndex,
                                  FIELD_GOAL_DEPTH_CM_ESTIMATED,
                                  FIELD_GOAL_WIDTH_CM_ESTIMATED)) {
                Chess_MarkScored(mgr, (uint8_t)fsm->currentTargetIndex);
                fsm->pushSuccesses++;
                Match_EnterState(fsm, MATCH_STATE_SCORED, nowMs);
                break;
            }

            /* 推丢判定：棋子已偏离推送轴线。 */
            if (Match_PushLateralOffsetCm(mgr, target, pose) >
                fsm->config.pushLostDistanceCm) {
                fsm->pushLostCount++;
                fsm->currentTargetIndex = -1;
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
                break;
            }

            if (Match_ElapsedInState(fsm, nowMs) > fsm->config.pushTimeoutMs) {
                fsm->pushLostCount++;
                fsm->currentTargetIndex = -1;
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
            }
            break;

        case MATCH_STATE_SCORED:
            /* 记分已在 PUSH 中完成，这里立即转入退出动作。 */
            fsm->currentTargetIndex = -1;
            Match_EnterState(fsm, MATCH_STATE_RETREAT, nowMs);
            break;

        case MATCH_STATE_RETREAT: {
            /*
             * 得分后必须退出球门区。
             * 否则车停在自己门口，会挡住后续棋子进门，等于自己堵自己。
             */
            float retreatX, retreatY;

            if (mgr->ourSide == FIELD_SIDE_BLUE) {
                retreatX = fsm->config.retreatDistanceCm + FIELD_GOAL_DEPTH_CM_ESTIMATED;
            } else {
                retreatX = FIELD_LENGTH_CM - FIELD_GOAL_DEPTH_CM_ESTIMATED
                         - fsm->config.retreatDistanceCm;
            }
            retreatY = FIELD_GOAL_CENTER_Y_CM;

            Navigator_SetTarget(nav, retreatX, retreatY, false);

            if (Navigator_HasArrived(nav) ||
                (Match_ElapsedInState(fsm, nowMs) > 4000U)) {
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
            }
            break;
        }

        case MATCH_STATE_RECOVER:
            /*
             * 恢复动作：倒车离开可能的卡死位置，给贴边校正创造机会。
             * 真正的位姿修复由 PoseEstimator_CorrectByWall 完成，
             * 这里只负责把车挪到能看见挡板的地方。
             */
            Navigator_SetTarget(nav, FIELD_PILE_CENTER_X_CM,
                                FIELD_PILE_CENTER_Y_CM, false);
            if (Navigator_HasArrived(nav) ||
                (Match_ElapsedInState(fsm, nowMs) > 5000U)) {
                Match_EnterState(fsm, MATCH_STATE_SEARCH, nowMs);
            }
            break;

        case MATCH_STATE_FINISHED:
        default:
            Navigator_Reset(nav);
            break;
    }

    /* 避障由 Navigator 内部处理，这里只记录对手过近的事实供诊断。 */
    (void)opponentDistCm;
}

Match_State Match_GetState(const Match_Fsm *fsm)
{
    return (fsm != 0) ? fsm->state : MATCH_STATE_INIT;
}

const char *Match_StateName(Match_State state)
{
    switch (state) {
        case MATCH_STATE_INIT:     return "INIT";
        case MATCH_STATE_SEARCH:   return "SEARCH";
        case MATCH_STATE_APPROACH: return "APPR";
        case MATCH_STATE_ALIGN:    return "ALIGN";
        case MATCH_STATE_PUSH:     return "PUSH";
        case MATCH_STATE_SCORED:   return "SCORED";
        case MATCH_STATE_RETREAT:  return "RETR";
        case MATCH_STATE_RECOVER:  return "RECOV";
        case MATCH_STATE_FINISHED: return "FIN";
        default:                   return "?";
    }
}
