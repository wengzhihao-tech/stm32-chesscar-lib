/**
 * @file selftest_match.c
 * @brief 赛题专用模块的 PC 端自测：位姿估计、棋子决策、导航、状态机、视觉协议。
 *
 * 这些是本赛题新增的算法，全部平台无关，可以在 PC 上直接验证逻辑，
 * 不需要烧板也不需要真实场地。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "chess_target.h"
#include "field_config.h"
#include "match_fsm.h"
#include "navigator.h"
#include "pose_estimator.h"
#include "vision_chess.h"

static int fails = 0;
#define CHECK(c, msg) do { \
    if (!(c)) { printf("FAIL: %s\n", (msg)); fails++; } \
    else { printf("ok  : %s\n", (msg)); } } while (0)

/* ================= 位姿估计 ================= */
static void test_pose(void)
{
    PoseEstimator pe;
    PoseEstimator_Config cfg;
    Pose2D start = { 20.0f, 100.0f, 0.0f };

    printf("\n--- 位姿估计 ---\n");
    PoseEstimator_LoadDefaultConfig(&cfg);
    cfg.useImuHeading = false; /* 先测纯里程计 */
    cfg.wheelTrackCm = 12.0f;
    CHECK(PoseEstimator_Init(&pe, &cfg, &start), "位姿估计初始化");

    /* 直线前进 50cm：两轮里程相同，航向应保持，X 增加 50。 */
    PoseEstimator_UpdateOdometry(&pe, 0.0f, 0.0f, 0.0f);   /* 首拍建基准 */
    PoseEstimator_UpdateOdometry(&pe, 50.0f, 50.0f, 0.0f);
    printf("直行后: x=%.2f y=%.2f th=%.2f\n", pe.pose.x, pe.pose.y, pe.pose.theta);
    CHECK(fabsf(pe.pose.x - 70.0f) < 0.1f, "直行 50cm 后 X 正确");
    CHECK(fabsf(pe.pose.y - 100.0f) < 0.1f, "直行不改变 Y");
    CHECK(fabsf(pe.pose.theta) < 0.1f, "直行不改变航向");

    /* 原地右转：左轮前进、右轮后退等量，位置不变、航向变负。 */
    PoseEstimator_Init(&pe, &cfg, &start);
    PoseEstimator_UpdateOdometry(&pe, 0.0f, 0.0f, 0.0f);
    PoseEstimator_UpdateOdometry(&pe, 5.0f, -5.0f, 0.0f);
    printf("原地转后: x=%.2f y=%.2f th=%.2f\n", pe.pose.x, pe.pose.y, pe.pose.theta);
    CHECK(fabsf(pe.pose.x - 20.0f) < 0.2f, "原地转向 X 几乎不变");
    CHECK(pe.pose.theta < -0.1f, "左轮快于右轮 → 航向为负(右转)");

    /* 角度归一化 */
    CHECK(fabsf(Pose_NormalizeAngleDeg(370.0f) - 10.0f) < 1e-3f, "370° 归一化为 10°");
    CHECK(fabsf(Pose_NormalizeAngleDeg(-190.0f) - 170.0f) < 1e-3f, "-190° 归一化为 170°");
    CHECK(fabsf(Pose_NormalizeAngleDeg(180.0f) - 180.0f) < 1e-3f, "180° 保持 180°");

    /* 大角度一次收敛（验证没用 while 循环慢慢减） */
    CHECK(fabsf(Pose_NormalizeAngleDeg(7200.0f + 45.0f) - 45.0f) < 1e-2f,
          "超大角度也能正确归一化");

    /* ---- 挡板贴边校正 ---- */
    {
        Pose2D p = { 100.0f, 100.0f, 180.0f }; /* 车头朝 -X，正对左墙 */
        PoseEstimator_Init(&pe, &cfg, &p);
        /* 左墙测距 10cm + 偏移 8cm → 车体中心应在 x=18 */
        CHECK(PoseEstimator_CorrectByWall(&pe, WALL_LEFT, 10.0f, 8.0f),
              "正对左墙时校正被接受");
        printf("校正后 x=%.2f (期望 18)\n", pe.pose.x);
        CHECK(fabsf(pe.pose.x - 18.0f) < 1e-3f, "左墙校正把 X 钉回绝对值");
        CHECK(pe.wallCorrectionCount == 1U, "校正计数递增");

        /* 航向偏差过大必须拒绝（斜测不等于垂直距离） */
        p.theta = 90.0f; /* 车头朝 +Y，与左墙法线差 90° */
        PoseEstimator_SetPose(&pe, &p);
        CHECK(!PoseEstimator_CorrectByWall(&pe, WALL_LEFT, 10.0f, 8.0f),
              "航向偏差过大时拒绝校正");
        CHECK(pe.rejectedCorrections >= 1U, "拒绝计数递增");

        /* 超量程必须拒绝（可能测到的是对方车而非挡板） */
        p.theta = 180.0f;
        PoseEstimator_SetPose(&pe, &p);
        CHECK(!PoseEstimator_CorrectByWall(&pe, WALL_LEFT, 200.0f, 8.0f),
              "超量程读数被拒绝");

        /* 右墙校正 */
        p.x = 100.0f; p.theta = 0.0f;
        PoseEstimator_SetPose(&pe, &p);
        PoseEstimator_CorrectByWall(&pe, WALL_RIGHT, 10.0f, 8.0f);
        printf("右墙校正后 x=%.2f (期望 %.2f)\n", pe.pose.x, FIELD_LENGTH_CM - 18.0f);
        CHECK(fabsf(pe.pose.x - (FIELD_LENGTH_CM - 18.0f)) < 1e-3f, "右墙校正正确");
    }

    /* 几何辅助函数 */
    {
        Pose2D a = { 0.0f, 0.0f, 0.0f };
        CHECK(fabsf(Pose_BearingToDeg(&a, 10.0f, 0.0f)) < 1e-3f, "正东方位角为 0");
        CHECK(fabsf(Pose_BearingToDeg(&a, 0.0f, 10.0f) - 90.0f) < 1e-3f, "正北方位角为 90");
        CHECK(fabsf(Pose_HeadingErrorDeg(&a, 0.0f, 10.0f) - 90.0f) < 1e-3f,
              "目标在左前方时航向误差为正(需左转)");
        CHECK(Pose_IsInsideField(&a, 5.0f) == false, "原点在安全边距外被判越界");
    }
}

/* ================= 棋子决策 ================= */
static void test_chess(void)
{
    Chess_TargetManager mgr;

    printf("\n--- 棋子决策 ---\n");
    Chess_InitManager(&mgr, FIELD_SIDE_BLUE);
    CHECK(fabsf(mgr.goalX) < 1e-6f, "甲队球门在 X=0");

    /* 分值表 */
    CHECK(Chess_LevelScore(CHESS_LEVEL_GENERAL) == 10U, "帅/将 10 分");
    CHECK(Chess_LevelScore(CHESS_LEVEL_SOLDIER) == 1U, "兵/卒 1 分");
    CHECK(Chess_LevelScore(CHESS_LEVEL_UNKNOWN) == 4U, "等级未知取保守中间值");

    /* 重复观测应合并而不是新增 */
    Chess_Observe(&mgr, 100.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, 1000U);
    int8_t i2 = Chess_Observe(&mgr, 101.0f, 100.5f, CHESS_COLOR_RED,
                              CHESS_LEVEL_HORSE, 1050U);
    printf("count after 2 obs = %u\n", mgr.count);
    CHECK(mgr.count == 1U, "近距离重复观测合并为同一枚");
    CHECK(i2 == 0, "返回同一槽位索引");

    /* 远处的是另一枚 */
    Chess_Observe(&mgr, 150.0f, 50.0f, CHESS_COLOR_BLACK, CHESS_LEVEL_CHARIOT, 1100U);
    CHECK(mgr.count == 2U, "远处观测新增槽位");

    /* 等级一旦识别出来，后续"未知"不应覆盖 */
    Chess_Observe(&mgr, 100.0f, 100.0f, CHESS_COLOR_UNKNOWN,
                  CHESS_LEVEL_UNKNOWN, 1200U);
    CHECK(Chess_GetPiece(&mgr, 0)->level == CHESS_LEVEL_HORSE,
          "已识别等级不被未知覆盖");

    /* 观测次数不足时不应被选为目标 */
    {
        Chess_TargetManager m2;
        Pose2D car = { 50.0f, 100.0f, 0.0f };
        Chess_InitManager(&m2, FIELD_SIDE_BLUE);
        Chess_Observe(&m2, 100.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_CHARIOT, 10U);
        CHECK(Chess_SelectBestTarget(&m2, &car, 2.0f, 10.0f) < 0,
              "单帧观测不足以成为目标(抗误检)");
        Chess_Observe(&m2, 100.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_CHARIOT, 20U);
        CHECK(Chess_SelectBestTarget(&m2, &car, 2.0f, 10.0f) >= 0,
              "两帧确认后可成为目标");
    }

    /* 高分棋子应优先于等距的低分棋子 */
    {
        Chess_TargetManager m3;
        Pose2D car = { 150.0f, 100.0f, 0.0f };
        int8_t best;
        Chess_InitManager(&m3, FIELD_SIDE_BLUE);
        /* 两枚棋子到车、到球门距离相同，只有分值不同 */
        Chess_Observe(&m3, 150.0f, 70.0f, CHESS_COLOR_RED, CHESS_LEVEL_SOLDIER, 10U);
        Chess_Observe(&m3, 150.0f, 70.0f, CHESS_COLOR_RED, CHESS_LEVEL_SOLDIER, 20U);
        Chess_Observe(&m3, 150.0f, 130.0f, CHESS_COLOR_RED, CHESS_LEVEL_GENERAL, 10U);
        Chess_Observe(&m3, 150.0f, 130.0f, CHESS_COLOR_RED, CHESS_LEVEL_GENERAL, 20U);
        best = Chess_SelectBestTarget(&m3, &car, 2.0f, 10.0f);
        printf("best=%d level=%s\n", best,
               Chess_LevelName(Chess_GetPiece(&m3, (uint8_t)best)->level));
        CHECK(Chess_GetPiece(&m3, (uint8_t)best)->level == CHESS_LEVEL_GENERAL,
              "等距时优先选高分棋子");
    }

    /* 近球门的棋子应优先于远球门的同分棋子 */
    {
        Chess_TargetManager m4;
        Pose2D car = { 150.0f, 100.0f, 0.0f };
        int8_t best;
        Chess_InitManager(&m4, FIELD_SIDE_BLUE); /* 球门在 X=0 */
        Chess_Observe(&m4, 60.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, 10U);
        Chess_Observe(&m4, 60.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, 20U);
        Chess_Observe(&m4, 240.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, 10U);
        Chess_Observe(&m4, 240.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, 20U);
        best = Chess_SelectBestTarget(&m4, &car, 2.0f, 10.0f);
        printf("best x=%.1f\n", Chess_GetPiece(&m4, (uint8_t)best)->x);
        CHECK(Chess_GetPiece(&m4, (uint8_t)best)->x < 100.0f,
              "同分时优先选离己方球门近的");
    }

    /* 推送起始点必须在棋子背向球门的一侧 */
    {
        float ax, ay;
        Chess_TargetManager m5;
        Chess_InitManager(&m5, FIELD_SIDE_BLUE); /* 球门 X=0 */
        Chess_Observe(&m5, 100.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, 10U);
        Chess_ComputeApproachPoint(&m5, 0U, 15.0f, &ax, &ay);
        printf("approach=(%.2f, %.2f) piece=(100,100) goal=(0,100)\n", ax, ay);
        CHECK(ax > 100.0f, "接近点在棋子远离球门的一侧");
        CHECK(fabsf(ax - 115.0f) < 0.1f, "接近点距棋子正好 standoff 距离");
        CHECK(fabsf(ay - 100.0f) < 0.1f, "共线情况下 Y 不偏移");
    }

    /* 进门判定：要求整体进入 */
    {
        Chess_TargetManager m6;
        Chess_InitManager(&m6, FIELD_SIDE_BLUE);
        Chess_Observe(&m6, 5.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, 10U);
        CHECK(Chess_IsInOurGoal(&m6, 0U, 30.0f, 60.0f), "深入球门内判定得分");
        /* 刚好压在边界上：因要求整体进入，应判未进 */
        m6.pieces[0].x = 30.0f;
        CHECK(!Chess_IsInOurGoal(&m6, 0U, 30.0f, 60.0f),
              "压线未整体进入时不算得分");
        /* Y 超出门宽 */
        m6.pieces[0].x = 5.0f;
        m6.pieces[0].y = 100.0f + 40.0f;
        CHECK(!Chess_IsInOurGoal(&m6, 0U, 30.0f, 60.0f), "超出门宽不算得分");
    }

    /* 记分与平局比较项 */
    {
        Chess_TargetManager m7;
        Chess_InitManager(&m7, FIELD_SIDE_BLUE);
        Chess_Observe(&m7, 100.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_GENERAL, 10U);
        Chess_MarkScored(&m7, 0U);
        CHECK(m7.ourScore == 10U, "记分累加分值");
        CHECK(m7.ourGoalCount == 1U, "进球总数累加(平局第一比较项)");
        CHECK(m7.ourGeneralScored, "帅/将归属被记录(平局第二比较项)");
        /* 重复记分不应重复加分 */
        Chess_MarkScored(&m7, 0U);
        CHECK(m7.ourScore == 10U, "已得分棋子不重复累加");
    }

    /* 老化剔除 */
    {
        Chess_TargetManager m8;
        Chess_InitManager(&m8, FIELD_SIDE_BLUE);
        Chess_Observe(&m8, 100.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, 1000U);
        CHECK(m8.count == 1U, "入表成功");
        Chess_AgeOut(&m8, 1000U + m8.pieceTimeoutMs + 1U);
        CHECK(m8.count == 0U, "超时未观测的棋子被剔除");
    }

    /* 乙队镜像 */
    {
        Chess_TargetManager m9;
        Chess_InitManager(&m9, FIELD_SIDE_RED);
        CHECK(fabsf(m9.goalX - FIELD_LENGTH_CM) < 1e-6f, "乙队球门在 X=300");
        Chess_Observe(&m9, 295.0f, 100.0f, CHESS_COLOR_BLACK, CHESS_LEVEL_HORSE, 10U);
        CHECK(Chess_IsInOurGoal(&m9, 0U, 30.0f, 60.0f), "乙队进门判定正确");
    }
}

/* ================= 导航 ================= */
static void test_navigator(void)
{
    Navigator nav;
    Navigator_Config cfg;
    Pose2D pose = { 100.0f, 100.0f, 0.0f };

    printf("\n--- 导航 ---\n");
    Navigator_LoadDefaultConfig(&cfg);
    CHECK(Navigator_Init(&nav, &cfg), "导航初始化");
    CHECK(cfg.pushSpeedCmps < cfg.cruiseSpeedCmps,
          "推送速度低于巡航速度(防止棋子被弹开)");

    /* 目标在正前方：应直行，两轮速度接近 */
    Navigator_SetTarget(&nav, 200.0f, 100.0f, false);
    Navigator_Update(&nav, &pose, 400.0f, 400.0f);
    printf("正前方: L=%.2f R=%.2f state=%d\n",
           nav.targetLeftCmps, nav.targetRightCmps, nav.state);
    CHECK(nav.state == NAV_STATE_DRIVING, "正对目标时进入行进状态");
    CHECK(fabsf(nav.targetLeftCmps - nav.targetRightCmps) < 1.0f,
          "正前方时两轮速度基本相同");
    CHECK(nav.targetLeftCmps > 0.0f, "前进速度为正");

    /* 目标在正后方：应原地转向 */
    Navigator_SetTarget(&nav, 0.0f, 100.0f, false);
    Navigator_Update(&nav, &pose, 400.0f, 400.0f);
    printf("正后方: L=%.2f R=%.2f state=%d\n",
           nav.targetLeftCmps, nav.targetRightCmps, nav.state);
    CHECK(nav.state == NAV_STATE_SPINNING, "大角度偏差时原地转向");
    CHECK(nav.targetLeftCmps * nav.targetRightCmps < 0.0f,
          "原地转向时两轮反向");

    /* 目标在左前方小角度：右轮应快于左轮 */
    Navigator_SetTarget(&nav, 200.0f, 115.0f, false);
    Navigator_Update(&nav, &pose, 400.0f, 400.0f);
    printf("左前方: L=%.2f R=%.2f err=%.2f\n",
           nav.targetLeftCmps, nav.targetRightCmps, nav.headingErrorDeg);
    CHECK(nav.headingErrorDeg > 0.0f, "左前方航向误差为正");
    CHECK(nav.targetRightCmps > nav.targetLeftCmps, "左转时右轮快于左轮");

    /* 到达判定 */
    {
        Pose2D at = { 200.0f, 100.0f, 0.0f };
        Navigator_SetTarget(&nav, 200.0f, 100.0f, false);
        Navigator_Update(&nav, &at, 400.0f, 400.0f);
        CHECK(Navigator_HasArrived(&nav), "到达目标点");
        CHECK(fabsf(nav.targetLeftCmps) < 1e-6f, "到达后停车");
    }

    /* 接近降速 */
    {
        Pose2D near = { 195.0f, 100.0f, 0.0f };
        Navigator_SetTarget(&nav, 200.0f, 100.0f, false);
        Navigator_Update(&nav, &near, 400.0f, 400.0f);
        printf("距目标 5cm: L=%.2f (巡航 %.1f)\n",
               nav.targetLeftCmps, cfg.cruiseSpeedCmps);
        CHECK(nav.targetLeftCmps < cfg.cruiseSpeedCmps, "接近目标时降速");
    }

    /* 推送模式速度更低 */
    {
        Pose2D far = { 100.0f, 100.0f, 0.0f };
        float cruiseSpeed, pushSpeed;
        Navigator_SetTarget(&nav, 280.0f, 100.0f, false);
        Navigator_Update(&nav, &far, 400.0f, 400.0f);
        cruiseSpeed = nav.targetLeftCmps;
        Navigator_SetTarget(&nav, 280.0f, 100.0f, true);
        Navigator_Update(&nav, &far, 400.0f, 400.0f);
        pushSpeed = nav.targetLeftCmps;
        printf("巡航=%.2f 推送=%.2f\n", cruiseSpeed, pushSpeed);
        CHECK(pushSpeed < cruiseSpeed, "推送模式使用更低速度");
    }

    /* 对手过近必须规避（赛题禁止恶意冲撞） */
    {
        Pose2D p = { 100.0f, 100.0f, 0.0f };
        Navigator_SetTarget(&nav, 200.0f, 100.0f, false);
        Navigator_Update(&nav, &p, 400.0f, 10.0f); /* 对手 10cm */
        printf("对手过近: L=%.2f R=%.2f avoid=%d\n",
               nav.targetLeftCmps, nav.targetRightCmps, nav.avoidingObstacle);
        CHECK(nav.avoidingObstacle, "检测到对手过近触发规避");
        CHECK(nav.targetLeftCmps * nav.targetRightCmps < 0.0f,
              "规避时原地转开而非继续前冲");
    }

    /* 推送模式下不应把被推的棋子当障碍物躲开 */
    {
        Pose2D p = { 100.0f, 100.0f, 0.0f };
        Navigator_SetTarget(&nav, 200.0f, 100.0f, true);
        Navigator_Update(&nav, &p, 5.0f, 400.0f); /* 前方 5cm 就是棋子 */
        CHECK(!nav.avoidingObstacle, "推送模式下忽略近距离前方障碍");
        CHECK(nav.targetLeftCmps > 0.0f, "推送模式继续前进");
    }
}

/* ================= 比赛状态机 ================= */
static void test_fsm(void)
{
    Match_Fsm fsm;
    Match_Config cfg;
    Chess_TargetManager mgr;
    PoseEstimator pe;
    PoseEstimator_Config peCfg;
    Navigator nav;
    Navigator_Config navCfg;
    Pose2D start = { 30.0f, 100.0f, 0.0f };
    uint32_t t = 0U;

    printf("\n--- 比赛状态机 ---\n");
    Match_LoadDefaultConfig(&cfg);
    CHECK(Match_Init(&fsm, &cfg), "状态机初始化");
    CHECK(Match_GetState(&fsm) == MATCH_STATE_INIT, "初始为 INIT");

    Chess_InitManager(&mgr, FIELD_SIDE_BLUE);
    PoseEstimator_LoadDefaultConfig(&peCfg);
    PoseEstimator_Init(&pe, &peCfg, &start);
    Navigator_LoadDefaultConfig(&navCfg);
    Navigator_Init(&nav, &navCfg);

    Match_Start(&fsm, t);
    CHECK(Match_GetState(&fsm) == MATCH_STATE_SEARCH, "发车后进入 SEARCH");

    /* 无棋子时应保持搜索并切换巡视点 */
    {
        uint8_t wp0 = fsm.searchWaypointIndex;
        t += cfg.searchTimeoutMs + 100U;
        Match_Update(&fsm, t, &mgr, &pe, &nav, 400.0f);
        printf("搜索超时后 waypoint %u -> %u\n", wp0, fsm.searchWaypointIndex);
        CHECK(fsm.searchWaypointIndex != wp0, "搜索超时后切换巡视点");
        CHECK(Match_GetState(&fsm) == MATCH_STATE_SEARCH, "仍在搜索状态");
    }

    /* 出现棋子 → 应转入接近 */
    Chess_Observe(&mgr, 120.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_CHARIOT, t);
    Chess_Observe(&mgr, 120.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_CHARIOT, t + 10U);
    t += 20U;
    Match_Update(&fsm, t, &mgr, &pe, &nav, 400.0f);
    printf("state=%s target=%d approach=(%.1f,%.1f)\n",
           Match_StateName(fsm.state), fsm.currentTargetIndex,
           fsm.approachX, fsm.approachY);
    CHECK(Match_GetState(&fsm) == MATCH_STATE_APPROACH, "发现棋子后进入 APPROACH");
    CHECK(fsm.currentTargetIndex >= 0, "锁定了目标棋子");
    CHECK(fsm.approachX > 120.0f, "接近点在棋子背向球门一侧");

    /* 接近超时应放弃该目标 */
    t += cfg.approachTimeoutMs + 100U;
    Match_Update(&fsm, t, &mgr, &pe, &nav, 400.0f);
    CHECK(Match_GetState(&fsm) == MATCH_STATE_SEARCH, "接近超时后回到搜索");
    CHECK(fsm.currentTargetIndex < 0, "超时后释放目标");

    /* 比赛时间到应结束 */
    t = fsm.matchStartMs + cfg.matchDurationMs + 1U;
    Match_Update(&fsm, t, &mgr, &pe, &nav, 400.0f);
    CHECK(Match_GetState(&fsm) == MATCH_STATE_FINISHED, "比赛时间到进入 FINISHED");

    /* 状态名齐全 */
    CHECK(strcmp(Match_StateName(MATCH_STATE_PUSH), "PUSH") == 0, "状态名正确");

    /* 巡视点镜像：乙队的点应在场地另一半 */
    {
        float bx, by, rx, ry;
        Match_Fsm f2;
        Match_Init(&f2, &cfg);
        Match_GetSearchWaypoint(&f2, FIELD_SIDE_BLUE, &bx, &by);
        Match_GetSearchWaypoint(&f2, FIELD_SIDE_RED, &rx, &ry);
        printf("waypoint blue=(%.1f,%.1f) red=(%.1f,%.1f)\n", bx, by, rx, ry);
        CHECK(fabsf((bx + rx) - FIELD_LENGTH_CM) < 1e-3f, "两阵营巡视点 X 镜像对称");
        CHECK(fabsf(by - ry) < 1e-3f, "巡视点 Y 相同");
    }

    /* 完整得分流程：接近→对准→推送→进门→记分→退出 */
    {
        Match_Fsm f3;
        Chess_TargetManager m3;
        PoseEstimator p3;
        Navigator n3;
        Pose2D pose3 = { 50.0f, 100.0f, 180.0f }; /* 车头朝球门(-X) */
        uint32_t tt = 1000U;
        int guard = 0;

        Match_Init(&f3, &cfg);
        Chess_InitManager(&m3, FIELD_SIDE_BLUE);
        PoseEstimator_Init(&p3, &peCfg, &pose3);
        Navigator_Init(&n3, &navCfg);
        Match_Start(&f3, tt);

        /* 棋子就在车前方、靠近球门 */
        Chess_Observe(&m3, 40.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_GENERAL, tt);
        Chess_Observe(&m3, 40.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_GENERAL, tt);
        tt += 20U;
        Match_Update(&f3, tt, &m3, &p3, &n3, 400.0f);
        CHECK(Match_GetState(&f3) == MATCH_STATE_APPROACH, "流程: 进入接近");

        /* 把车直接放到接近点并标记到达 */
        pose3.x = f3.approachX;
        pose3.y = f3.approachY;
        pose3.theta = 180.0f;
        PoseEstimator_SetPose(&p3, &pose3);
        Navigator_SetTarget(&n3, pose3.x, pose3.y, false);
        Navigator_Update(&n3, &pose3, 400.0f, 400.0f);
        tt += 20U;
        Match_Update(&f3, tt, &m3, &p3, &n3, 400.0f);
        printf("流程 state=%s\n", Match_StateName(f3.state));
        CHECK(Match_GetState(&f3) == MATCH_STATE_ALIGN, "流程: 到位后对准");

        /* 对准后应进入推送 */
        tt += 20U;
        Match_Update(&f3, tt, &m3, &p3, &n3, 400.0f);
        printf("流程 state=%s attempts=%u\n",
               Match_StateName(f3.state), f3.pushAttempts);
        CHECK(Match_GetState(&f3) == MATCH_STATE_PUSH, "流程: 对准后推送");
        CHECK(f3.pushAttempts == 1U, "推送尝试计数");

        /* 把棋子移入球门，应判定得分 */
        m3.pieces[0].x = 5.0f;
        tt += 20U;
        Match_Update(&f3, tt, &m3, &p3, &n3, 400.0f);
        printf("流程 state=%s score=%u goals=%u\n",
               Match_StateName(f3.state), m3.ourScore, m3.ourGoalCount);
        CHECK(f3.pushSuccesses == 1U, "流程: 推送成功计数");
        CHECK(m3.ourScore == 10U, "流程: 得分累加");
        CHECK(m3.ourGeneralScored, "流程: 帅/将归属记录");

        /* 得分后应退出球门区，避免挡住自己的门 */
        tt += 20U;
        Match_Update(&f3, tt, &m3, &p3, &n3, 400.0f);
        while ((Match_GetState(&f3) == MATCH_STATE_SCORED) && (guard++ < 5)) {
            tt += 20U;
            Match_Update(&f3, tt, &m3, &p3, &n3, 400.0f);
        }
        printf("流程 state=%s\n", Match_StateName(f3.state));
        CHECK(Match_GetState(&f3) == MATCH_STATE_RETREAT, "流程: 得分后退出球门");
    }

    /* 位姿越界应触发恢复 */
    {
        Match_Fsm f4;
        Chess_TargetManager m4;
        PoseEstimator p4;
        Navigator n4;
        Pose2D bad = { -50.0f, 100.0f, 0.0f }; /* 场地外，物理上不可能 */
        uint32_t tt = 500U;

        Match_Init(&f4, &cfg);
        Chess_InitManager(&m4, FIELD_SIDE_BLUE);
        PoseEstimator_Init(&p4, &peCfg, &bad);
        Navigator_Init(&n4, &navCfg);
        Match_Start(&f4, tt);
        Chess_Observe(&m4, 120.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, tt);
        Chess_Observe(&m4, 120.0f, 100.0f, CHESS_COLOR_RED, CHESS_LEVEL_HORSE, tt);
        tt += 20U;
        Match_Update(&f4, tt, &m4, &p4, &n4, 400.0f); /* → APPROACH */
        tt += 20U;
        Match_Update(&f4, tt, &m4, &p4, &n4, 400.0f); /* 检测越界 */
        printf("越界后 state=%s recover=%u\n",
               Match_StateName(f4.state), f4.recoverCount);
        CHECK(f4.recoverCount >= 1U, "位姿越界触发恢复计数");
    }
}

/* ================= 视觉协议 ================= */
static void test_vision(void)
{
    Vision_Frame tx;
    Vision_Frame rx;
    LinkProtocol_Frame frame;
    uint8_t payload[32];
    uint8_t len;

    printf("\n--- 视觉协议 ---\n");

    memset(&tx, 0, sizeof(tx));
    tx.count = 3U;
    tx.valid = true;
    tx.pieces[0].x = 100.0f; tx.pieces[0].y = 50.0f;
    tx.pieces[0].color = CHESS_COLOR_RED;
    tx.pieces[0].level = CHESS_LEVEL_GENERAL;
    tx.pieces[0].confidence = 90U;
    tx.pieces[1].x = 200.0f; tx.pieces[1].y = 150.0f;
    tx.pieces[1].color = CHESS_COLOR_BLACK;
    tx.pieces[1].level = CHESS_LEVEL_UNKNOWN;
    tx.pieces[1].confidence = 70U;
    tx.pieces[2].x = 60.0f; tx.pieces[2].y = 30.0f;
    tx.pieces[2].color = CHESS_COLOR_RED;
    tx.pieces[2].level = CHESS_LEVEL_SOLDIER;
    tx.pieces[2].confidence = 20U; /* 低置信度，应被丢弃 */

    len = Vision_BuildChessPayload(&tx, payload, sizeof(payload));
    printf("payload len=%u (期望 16)\n", len);
    CHECK(len == 16U, "payload 长度 = 1 + 3*5");

    /* 构造一帧并解析回来 */
    memset(&frame, 0, sizeof(frame));
    frame.type = LINK_MSG_CHESS_PIECES;
    frame.length = len;
    memcpy(frame.payload, payload, len);

    CHECK(Vision_ParseChessFrame(&frame, &rx), "解析棋子帧成功");
    printf("rx count=%u p0=(%.1f,%.1f) lvl=%s conf=%u\n",
           rx.count, rx.pieces[0].x, rx.pieces[0].y,
           Chess_LevelName(rx.pieces[0].level), rx.pieces[0].confidence);
    CHECK(rx.count == 3U, "棋子数量还原");
    CHECK(fabsf(rx.pieces[0].x - 100.0f) < 2.0f, "X 坐标还原(2cm 量化内)");
    CHECK(fabsf(rx.pieces[0].y - 50.0f) < 1.0f, "Y 坐标还原");
    CHECK(rx.pieces[0].level == CHESS_LEVEL_GENERAL, "等级还原");
    CHECK(rx.pieces[0].color == CHESS_COLOR_RED, "颜色还原");
    CHECK(rx.pieces[1].level == CHESS_LEVEL_UNKNOWN, "未知等级保持未知");

    /* 喂给管理器，低置信度应被过滤 */
    {
        Chess_TargetManager mgr;
        uint8_t accepted;
        Chess_InitManager(&mgr, FIELD_SIDE_BLUE);
        accepted = Vision_FeedToManager(&rx, &mgr, 50U, 1000U);
        printf("accepted=%u count=%u\n", accepted, mgr.count);
        CHECK(accepted == 2U, "低置信度观测被过滤");
        CHECK(mgr.count == 2U, "仅高置信度棋子入表");
    }

    /* 错误类型帧应被拒绝 */
    frame.type = 0x10U;
    CHECK(!Vision_ParseChessFrame(&frame, &rx), "非棋子帧被拒绝");

    /* 长度不足应被拒绝（声明 3 枚但只给了 1 枚的数据） */
    frame.type = LINK_MSG_CHESS_PIECES;
    frame.length = 6U;
    frame.payload[0] = 3U;
    CHECK(!Vision_ParseChessFrame(&frame, &rx), "长度与数量不符的帧被拒绝");

    /* 场地外坐标应被过滤 */
    {
        Chess_TargetManager mgr;
        Vision_Frame bad;
        memset(&bad, 0, sizeof(bad));
        bad.count = 1U; bad.valid = true;
        bad.pieces[0].x = 500.0f; /* 超出 300cm */
        bad.pieces[0].y = 100.0f;
        bad.pieces[0].confidence = 99U;
        Chess_InitManager(&mgr, FIELD_SIDE_BLUE);
        CHECK(Vision_FeedToManager(&bad, &mgr, 50U, 1000U) == 0U,
              "场地外坐标被过滤");
    }
}

int main(void)
{
    printf("===== 赛题1 专用模块自测 =====\n");
    test_pose();
    test_chess();
    test_navigator();
    test_fsm();
    test_vision();
    printf("\n==== %s (fails=%d) ====\n",
           fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
