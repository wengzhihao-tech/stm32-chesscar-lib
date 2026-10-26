/*
 * common/motion_ramp.c 的 PC 端自测。
 *
 * 独立成一个可执行文件，便于只跑斜坡相关用例：
 *     make run-ramp
 *
 * 这些断言全部是纯数值逻辑，不需要上车。但斜坡的五个物理参数
 * （accel / decel / angularRate / minMove / reverseHoldCycles）
 * 必须现场标定：数值逻辑正确不等于参数合适。
 * 标定步骤见 docs/MOTION_RAMP.md。
 */
#include <stdio.h>
#include <math.h>
#include "motion_ramp.h"
#include "encoder_math.h"
#include "car_physical_config.h"

static int fails = 0;
#define CHECK(c,msg) do{ if(!(c)){printf("FAIL: %s\n",msg);fails++;} else printf("ok  : %s\n",msg);}while(0)

int main(void)
{
    Encoder_Config ec = { CAR_WHEEL_DIAMETER_CM, CAR_GEAR_RATIO,
                          CAR_ENCODER_COUNTS_PER_MOTOR_REV,
                          (float)CAR_SPEED_LOOP_PERIOD_MS };
    MotionRamp_Config rc;
    MotionRamp_State rs;

    printf("--- 运动斜坡：初始化与参数校验 ---\n");
    MotionRamp_LoadDefaultConfig(&rc);
    rc.periodMs = (float)CAR_SPEED_LOOP_PERIOD_MS;
    CHECK(MotionRamp_Init(&rs, &rc), "斜坡初始化成功");
    {
        MotionRamp_Config bad = rc;
        bad.periodMs = 0.0f;
        CHECK(!MotionRamp_Init(&rs, &bad), "周期为 0 被拒绝");
        bad = rc; bad.accelCmps2 = -1.0f;
        CHECK(!MotionRamp_Init(&rs, &bad), "负加速度被拒绝");
        CHECK(!MotionRamp_Init(0, &rc) && !MotionRamp_Init(&rs, 0), "NULL 入参被拒绝");
    }
    MotionRamp_Init(&rs, &rc);

    printf("\n--- 软启动斜坡 ---\n");
    /* 默认 60cm/s^2 @10ms → 单拍 0.6cm/s */
    MotionRamp_SetTarget(&rs, 30.0f, 0.0f);
    MotionRamp_Update(&rs);
    printf("第 1 拍 linear=%.3f cm/s\n", MotionRamp_GetLinearCmps(&rs));
    CHECK(fabsf(MotionRamp_GetLinearCmps(&rs) - 0.6f) < 1e-4f, "单拍加速步长 = accel * T");
    CHECK(!MotionRamp_IsSettled(&rs), "未到目标不算 settled");
    {
        int cycles = 1;
        while (!MotionRamp_IsSettled(&rs) && cycles < 10000) {
            MotionRamp_Update(&rs);
            cycles++;
        }
        printf("settled after %d 拍 (%.0f ms) at %.3f cm/s\n",
               cycles, cycles * rc.periodMs, MotionRamp_GetLinearCmps(&rs));
        CHECK(cycles == 50, "0->30cm/s 恰好 50 拍 (500ms)");
    }
    CHECK(fabsf(MotionRamp_GetLinearCmps(&rs) - 30.0f) < 1e-4f, "斜坡末端精确落在目标上不过冲");

    printf("\n--- 低速不可分辨下限 ---\n");
    /*
     * 下限必须高于编码器量化步长，否则速度环只能在不可分辨区里抖。
     * 这是 M 法测速的原理限制，不是调 PID 能解决的。
     * 这条断言故意不写死 1.42，而是跟着机械参数算，换车后依然有效。
     */
    {
        float quantum = Encoder_SpeedQuantumCmps(&ec);
        printf("量化步长 %.4f cm/s，minMoveCmps %.2f cm/s (%.1f 倍)\n",
               quantum, rc.minMoveCmps, rc.minMoveCmps / quantum);
        CHECK(quantum > 0.0f, "量化步长可计算");
        CHECK(rc.minMoveCmps > 2.0f * quantum, "默认下限不低于 2 个量化步长");
    }
    MotionRamp_Init(&rs, &rc);
    MotionRamp_SetTarget(&rs, 2.0f, 0.0f);
    MotionRamp_Update(&rs);
    CHECK(MotionRamp_GetLinearCmps(&rs) == 0.0f, "低于下限的线速度目标被归零");
    MotionRamp_SetTarget(&rs, 0.0f, 8.0f);
    MotionRamp_Update(&rs);
    CHECK(MotionRamp_GetAngularCmps(&rs) > 0.0f, "角速度不受线速度下限影响(原地对准要用)");

    printf("\n--- 换向：先过零再反向 ---\n");
    MotionRamp_Init(&rs, &rc);
    MotionRamp_SetTarget(&rs, 20.0f, 0.0f);
    {
        int i;
        for (i = 0; i < 200; i++) { MotionRamp_Update(&rs); }
    }
    CHECK(fabsf(MotionRamp_GetLinearCmps(&rs) - 20.0f) < 1e-4f, "先达到 +20cm/s");
    MotionRamp_SetTarget(&rs, -20.0f, 0.0f);
    {
        float prev = MotionRamp_GetLinearCmps(&rs);
        int zeroHold = 0, jumpThroughZero = 0, sawReversing = 0, i;
        /* 逐拍检查符号翻转，而不是只看终值。 */
        for (i = 0; i < 400; i++) {
            float v;
            MotionRamp_Update(&rs);
            v = MotionRamp_GetLinearCmps(&rs);
            if (MotionRamp_IsReversing(&rs)) { sawReversing = 1; }
            if (prev > 0.0f && v < 0.0f) { jumpThroughZero = 1; }
            if (v == 0.0f) { zeroHold++; }
            prev = v;
        }
        printf("换向: 零速保持 %d 拍, 跨零跳变 %d, 终值 %.3f cm/s\n",
               zeroHold, jumpThroughZero, prev);
        CHECK(sawReversing, "换向期间 IsReversing 为真");
        CHECK(!jumpThroughZero, "换向不会一拍内跨过零点(避免 H 桥穿通与机械冲击)");
        CHECK(zeroHold >= (int)rc.reverseHoldCycles, "零速至少保持配置的拍数");
        CHECK(fabsf(prev + 20.0f) < 1e-4f, "换向流程结束后到达 -20cm/s");
    }

    printf("\n--- 减速斜率独立于加速斜率 ---\n");
    MotionRamp_Init(&rs, &rc);
    MotionRamp_SetTarget(&rs, 40.0f, 0.0f);
    {
        int i;
        for (i = 0; i < 500; i++) { MotionRamp_Update(&rs); }
    }
    MotionRamp_SetTarget(&rs, 10.0f, 0.0f);
    MotionRamp_Update(&rs);
    printf("减速第 1 拍 -> %.3f cm/s (期望 38.8)\n", MotionRamp_GetLinearCmps(&rs));
    CHECK(fabsf(MotionRamp_GetLinearCmps(&rs) - 38.8f) < 1e-4f, "减速走 decel 斜率而非 accel");

    printf("\n--- 急停与角速度限幅 ---\n");
    MotionRamp_EmergencyStop(&rs);
    CHECK(MotionRamp_GetLinearCmps(&rs) == 0.0f && MotionRamp_GetAngularCmps(&rs) == 0.0f,
          "急停立刻清零不走斜坡");
    CHECK(!MotionRamp_IsReversing(&rs), "急停清除换向状态");
    MotionRamp_Init(&rs, &rc);
    MotionRamp_SetTarget(&rs, 0.0f, 45.0f);
    MotionRamp_Update(&rs);
    printf("角速度第 1 拍 -> %.3f cm/s (期望 0.9)\n", MotionRamp_GetAngularCmps(&rs));
    CHECK(fabsf(MotionRamp_GetAngularCmps(&rs) - 0.9f) < 1e-4f,
          "角速度单拍步长 = angularRate * T");

    /* 所有接口都要能吃 NULL：中断里误调也不能跑飞。 */
    MotionRamp_Update(0);
    MotionRamp_SetTarget(0, 1.0f, 1.0f);
    MotionRamp_Reset(0);
    MotionRamp_EmergencyStop(0);
    CHECK(MotionRamp_GetLinearCmps(0) == 0.0f &&
          MotionRamp_GetAngularCmps(0) == 0.0f &&
          !MotionRamp_IsSettled(0) &&
          !MotionRamp_IsReversing(0), "NULL 入参安全");

    printf("\n==== %s (fails=%d) ====\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
