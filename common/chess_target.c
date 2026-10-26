/**
 * @file chess_target.c
 * @brief 棋子管理与目标决策实现。
 */
#include "chess_target.h"

#include <math.h>

uint8_t Chess_LevelScore(Chess_Level level)
{
    /*
     * ⚠ 假定分值。赛题只规定"按棋子等级设置 1-10 分差异化分值"，
     * 未公布具体对应表。此处按象棋常规强弱排序给出，
     * 拿到主办方正式计分表后必须更新。
     */
    switch (level) {
        case CHESS_LEVEL_GENERAL:  return 10U; /* 帅/将，兼有平局抢分价值 */
        case CHESS_LEVEL_CHARIOT:  return 8U;  /* 车 */
        case CHESS_LEVEL_CANNON:   return 6U;  /* 炮 */
        case CHESS_LEVEL_HORSE:    return 5U;  /* 马 */
        case CHESS_LEVEL_ELEPHANT: return 3U;  /* 象/相 */
        case CHESS_LEVEL_GUARD:    return 3U;  /* 士/仕 */
        case CHESS_LEVEL_SOLDIER:  return 1U;  /* 兵/卒 */
        case CHESS_LEVEL_UNKNOWN:
        default:
            /*
             * 等级识别失败时给中间值 4：
             * 既不会因为"可能是高分子"而盲目优先，
             * 也不会因为"可能是低分子"而放弃——毕竟推进球门就有分。
             */
            return 4U;
    }
}

const char *Chess_LevelName(Chess_Level level)
{
    switch (level) {
        case CHESS_LEVEL_GENERAL:  return "JIANG";
        case CHESS_LEVEL_CHARIOT:  return "CHE";
        case CHESS_LEVEL_CANNON:   return "PAO";
        case CHESS_LEVEL_HORSE:    return "MA";
        case CHESS_LEVEL_ELEPHANT: return "XIANG";
        case CHESS_LEVEL_GUARD:    return "SHI";
        case CHESS_LEVEL_SOLDIER:  return "BING";
        case CHESS_LEVEL_UNKNOWN:
        default:                   return "UNK";
    }
}

void Chess_SetOurSide(Chess_TargetManager *mgr, uint8_t ourSide)
{
    if (mgr == 0) {
        return;
    }

    mgr->ourSide = ourSide;
    /* 甲队（蓝）球门在 X≈0，乙队（红）球门在 X≈300。 */
    mgr->goalX = (ourSide == FIELD_SIDE_BLUE) ? 0.0f : FIELD_LENGTH_CM;
    mgr->goalY = FIELD_GOAL_CENTER_Y_CM;
}

void Chess_InitManager(Chess_TargetManager *mgr, uint8_t ourSide)
{
    uint8_t i;

    if (mgr == 0) {
        return;
    }

    for (i = 0U; i < CHESS_MAX_TRACKED_PIECES; ++i) {
        mgr->pieces[i].valid = false;
        mgr->pieces[i].scored = false;
        mgr->pieces[i].observeCount = 0U;
        mgr->pieces[i].color = CHESS_COLOR_UNKNOWN;
        mgr->pieces[i].level = CHESS_LEVEL_UNKNOWN;
        mgr->pieces[i].x = 0.0f;
        mgr->pieces[i].y = 0.0f;
        mgr->pieces[i].lastSeenMs = 0U;
    }

    mgr->count = 0U;
    mgr->pieceTimeoutMs = 3000U;  /* 3 秒未见则剔除 */
    /*
     * 合并半径取 6cm：棋子直径约 3.2cm，视觉定位误差常有 2~3cm，
     * 取 6cm 能把同一枚的抖动观测合并，又不会把相邻两枚棋子合成一枚。
     */
    mgr->mergeRadiusCm = 6.0f;
    mgr->minObserveCount = 2U;    /* 至少两帧，过滤单帧误检 */
    mgr->ourScore = 0U;
    mgr->ourGoalCount = 0U;
    mgr->ourGeneralScored = false;

    Chess_SetOurSide(mgr, ourSide);
}

int8_t Chess_Observe(Chess_TargetManager *mgr, float x, float y,
                     Chess_Color color, Chess_Level level, uint32_t nowMs)
{
    uint8_t i;
    int8_t freeSlot = -1;
    float bestDist = 0.0f;
    int8_t bestIdx = -1;

    if (mgr == 0) {
        return -1;
    }

    /* 先找最近的已有棋子，判断是否为同一枚的重复观测。 */
    for (i = 0U; i < CHESS_MAX_TRACKED_PIECES; ++i) {
        if (!mgr->pieces[i].valid) {
            if (freeSlot < 0) {
                freeSlot = (int8_t)i;
            }
            continue;
        }
        if (mgr->pieces[i].scored) {
            continue;
        }

        float dx = mgr->pieces[i].x - x;
        float dy = mgr->pieces[i].y - y;
        float d = sqrtf(dx * dx + dy * dy);
        if ((bestIdx < 0) || (d < bestDist)) {
            bestDist = d;
            bestIdx = (int8_t)i;
        }
    }

    /* 命中已有棋子：更新位置并累加观测次数。 */
    if ((bestIdx >= 0) && (bestDist <= mgr->mergeRadiusCm)) {
        Chess_Piece *p = &mgr->pieces[bestIdx];
        /*
         * 位置用低通平滑，避免视觉抖动让目标点来回跳，
         * 否则车会在接近过程中反复修正航向。
         */
        p->x = ChessCar_LowPass(p->x, x, 0.4f);
        p->y = ChessCar_LowPass(p->y, y, 0.4f);
        if (color != CHESS_COLOR_UNKNOWN) {
            p->color = color;
        }
        /* 等级一旦识别出来就保留，不让后续的"未知"把它覆盖掉。 */
        if (level != CHESS_LEVEL_UNKNOWN) {
            p->level = level;
        }
        p->lastSeenMs = nowMs;
        if (p->observeCount < 255U) {
            p->observeCount++;
        }
        return bestIdx;
    }

    /* 新棋子入表。 */
    if (freeSlot < 0) {
        return -1; /* 表满。32 枚是赛题上限，正常不会满。 */
    }

    mgr->pieces[freeSlot].valid = true;
    mgr->pieces[freeSlot].scored = false;
    mgr->pieces[freeSlot].x = x;
    mgr->pieces[freeSlot].y = y;
    mgr->pieces[freeSlot].color = color;
    mgr->pieces[freeSlot].level = level;
    mgr->pieces[freeSlot].lastSeenMs = nowMs;
    mgr->pieces[freeSlot].observeCount = 1U;
    mgr->count++;
    return freeSlot;
}

void Chess_AgeOut(Chess_TargetManager *mgr, uint32_t nowMs)
{
    uint8_t i;

    if (mgr == 0) {
        return;
    }

    for (i = 0U; i < CHESS_MAX_TRACKED_PIECES; ++i) {
        if (!mgr->pieces[i].valid || mgr->pieces[i].scored) {
            continue;
        }
        /* 用减法比较，这样 nowMs 回绕时也不会误判。 */
        if ((uint32_t)(nowMs - mgr->pieces[i].lastSeenMs) > mgr->pieceTimeoutMs) {
            mgr->pieces[i].valid = false;
            mgr->pieces[i].observeCount = 0U;
            if (mgr->count > 0U) {
                mgr->count--;
            }
        }
    }
}

void Chess_MarkScored(Chess_TargetManager *mgr, uint8_t index)
{
    if ((mgr == 0) || (index >= CHESS_MAX_TRACKED_PIECES)) {
        return;
    }
    if (!mgr->pieces[index].valid || mgr->pieces[index].scored) {
        return;
    }

    mgr->pieces[index].scored = true;
    mgr->ourScore += Chess_LevelScore(mgr->pieces[index].level);
    mgr->ourGoalCount++;
    if (mgr->pieces[index].level == CHESS_LEVEL_GENERAL) {
        mgr->ourGeneralScored = true; /* 平局第二比较项 */
    }
    if (mgr->count > 0U) {
        mgr->count--;
    }
}

int8_t Chess_SelectBestTarget(const Chess_TargetManager *mgr,
                              const Pose2D *carPose,
                              float pushDistanceWeight,
                              float scoreWeight)
{
    uint8_t i;
    int8_t bestIdx = -1;
    float bestCost = 0.0f;

    if ((mgr == 0) || (carPose == 0)) {
        return -1;
    }

    for (i = 0U; i < CHESS_MAX_TRACKED_PIECES; ++i) {
        const Chess_Piece *p = &mgr->pieces[i];
        float approachDist, pushDist, cost, risk;
        float dx, dy;

        if (!p->valid || p->scored) {
            continue;
        }
        /* 观测次数不足的先不当目标，避免追着误检跑。 */
        if (p->observeCount < mgr->minObserveCount) {
            continue;
        }

        /* 车到棋子的空载接近距离。 */
        dx = p->x - carPose->x;
        dy = p->y - carPose->y;
        approachDist = sqrtf(dx * dx + dy * dy);

        /* 棋子到己方球门的推送距离——这段路要推着棋子走，代价更高。 */
        dx = mgr->goalX - p->x;
        dy = mgr->goalY - p->y;
        pushDist = sqrtf(dx * dx + dy * dy);

        /*
         * 风险惩罚：
         * 1. 贴墙棋子难以从背后对正（车可能没有空间站到棋子后方）
         * 2. 离对方球门太近的棋子，推回来路程长且容易被对方反抢
         */
        risk = 0.0f;
        if ((p->x < FIELD_WALL_SAFE_MARGIN_CM) ||
            (p->x > (FIELD_LENGTH_CM - FIELD_WALL_SAFE_MARGIN_CM)) ||
            (p->y < FIELD_WALL_SAFE_MARGIN_CM) ||
            (p->y > (FIELD_WIDTH_CM - FIELD_WALL_SAFE_MARGIN_CM))) {
            risk += 25.0f;
        }

        cost = approachDist
             + pushDist * pushDistanceWeight
             - (float)Chess_LevelScore(p->level) * scoreWeight
             + risk;

        if ((bestIdx < 0) || (cost < bestCost)) {
            bestCost = cost;
            bestIdx = (int8_t)i;
        }
    }

    return bestIdx;
}

void Chess_ComputeApproachPoint(const Chess_TargetManager *mgr, uint8_t index,
                                float standoffCm, float *outX, float *outY)
{
    const Chess_Piece *p;
    float dx, dy, len;

    if ((mgr == 0) || (index >= CHESS_MAX_TRACKED_PIECES) ||
        (outX == 0) || (outY == 0)) {
        return;
    }

    p = &mgr->pieces[index];

    /*
     * 从球门指向棋子的单位向量，沿它再往外延伸 standoffCm，
     * 得到的点就在"棋子背向球门的一侧"。
     * 车开到这里、车头朝球门，推过去才是直线把棋子送进门。
     */
    dx = p->x - mgr->goalX;
    dy = p->y - mgr->goalY;
    len = sqrtf(dx * dx + dy * dy);

    if (len < 1e-3f) {
        /* 棋子几乎就在球门中心，不需要再接近。 */
        *outX = p->x;
        *outY = p->y;
        return;
    }

    *outX = p->x + (dx / len) * standoffCm;
    *outY = p->y + (dy / len) * standoffCm;
}

bool Chess_IsInOurGoal(const Chess_TargetManager *mgr, uint8_t index,
                       float goalDepthCm, float goalWidthCm)
{
    const Chess_Piece *p;
    float halfWidth;
    float pieceRadiusCm;

    if ((mgr == 0) || (index >= CHESS_MAX_TRACKED_PIECES)) {
        return false;
    }

    p = &mgr->pieces[index];
    if (!p->valid) {
        return false;
    }

    halfWidth = goalWidthCm * 0.5f;
    /*
     * 赛题要求棋子**整体**进入球门区，所以用棋子半径做内缩判断：
     * 棋子中心必须比边界再深入一个半径，整体才算进门。
     */
    pieceRadiusCm = (CHESS_PIECE_DIAMETER_MM * 0.1f) * 0.5f;

    if (CHESSCAR_ABS(p->y - mgr->goalY) > (halfWidth - pieceRadiusCm)) {
        return false;
    }

    if (mgr->ourSide == FIELD_SIDE_BLUE) {
        return p->x <= (goalDepthCm - pieceRadiusCm);
    }
    return p->x >= (FIELD_LENGTH_CM - goalDepthCm + pieceRadiusCm);
}

const Chess_Piece *Chess_GetPiece(const Chess_TargetManager *mgr, uint8_t index)
{
    if ((mgr == 0) || (index >= CHESS_MAX_TRACKED_PIECES)) {
        return 0;
    }
    return &mgr->pieces[index];
}
