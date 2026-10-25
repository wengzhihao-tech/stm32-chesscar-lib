/**
 * @file speed_control.c
 * @brief 双轮速度闭环实现：差分测速 → 低通 → 前馈 + PID → 死区补偿 → 限幅输出。
 */
#include "speed_control.h"

#include "car_physical_config.h"

#define SPEED_CONTROL_DEFAULT_FILTER_K (0.5f)

void SpeedControl_LoadDefaultConfig(SpeedControl_Config *config)
{
    if (config == 0) {
        return;
    }

    config->encoder.wheelDiameterCm = CAR_WHEEL_DIAMETER_CM;
    config->encoder.gearRatio = CAR_GEAR_RATIO;
    config->encoder.pulsesPerRound = CAR_ENCODER_COUNTS_PER_MOTOR_REV;
    config->encoder.samplePeriodMs = (float)CAR_SPEED_LOOP_PERIOD_MS;

    /*
     * 初始增益取保守值：先只开 P 跑通方向与量级，再逐步加 I 消静差。
     * 速度环上 D 项对编码器量化噪声极其敏感，默认关闭。
     */
    config->leftKp = 18.0f;
    config->leftKi = 6.0f;
    config->leftKd = 0.0f;
    config->rightKp = 18.0f;
    config->rightKi = 6.0f;
    config->rightKd = 0.0f;

    config->integralLimit = (float)CAR_PWM_OUTPUT_LIMIT;
    config->outputLimit = (int16_t)CAR_PWM_OUTPUT_LIMIT;

    config->leftFeedforwardPwmPerCmps = 0.0f;
    config->rightFeedforwardPwmPerCmps = 0.0f;
    config->leftDeadzonePwm = CAR_MOTOR_STARTUP_DEADZONE_PWM;
    config->rightDeadzonePwm = CAR_MOTOR_STARTUP_DEADZONE_PWM;
    config->feedbackFilterK = SPEED_CONTROL_DEFAULT_FILTER_K;

    config->directionFaultMinTargetCmps = 5.0f;
    config->directionFaultMinFeedbackCmps = 3.0f;
    config->overspeedLimitCmps = CAR_THEORETICAL_NO_LOAD_CMPS * 1.2f;
    config->directionFaultConfirmCycles = 25U; /* 10ms × 25 = 250ms */
    config->overspeedConfirmCycles = 10U;

    config->leftEncoderSign = 1;
    config->rightEncoderSign = 1;
    config->leftMotorSign = 1;
    config->rightMotorSign = 1;

    config->readLeftEncoder = 0;
    config->readRightEncoder = 0;
    config->setOutput = 0;
}

static int8_t SpeedControl_NormalizeSign(int8_t sign)
{
    return (sign < 0) ? -1 : 1;
}

bool SpeedControl_Init(SpeedControl_Controller *c, const SpeedControl_Config *config)
{
    if ((c == 0) || (config == 0) ||
        (config->readLeftEncoder == 0) || (config->readRightEncoder == 0) ||
        (config->setOutput == 0) || (config->outputLimit <= 0) ||
        (config->encoder.samplePeriodMs <= 0.0f)) {
        return false;
    }

    c->config = *config;
    c->config.leftEncoderSign = SpeedControl_NormalizeSign(config->leftEncoderSign);
    c->config.rightEncoderSign = SpeedControl_NormalizeSign(config->rightEncoderSign);
    c->config.leftMotorSign = SpeedControl_NormalizeSign(config->leftMotorSign);
    c->config.rightMotorSign = SpeedControl_NormalizeSign(config->rightMotorSign);
    if (c->config.feedbackFilterK <= 0.0f || c->config.feedbackFilterK > 1.0f) {
        c->config.feedbackFilterK = SPEED_CONTROL_DEFAULT_FILTER_K;
    }

    PID_Init(&c->leftPid, c->config.leftKp, c->config.leftKi, c->config.leftKd,
             c->config.integralLimit, (float)c->config.outputLimit);
    PID_Init(&c->rightPid, c->config.rightKp, c->config.rightKi, c->config.rightKd,
             c->config.integralLimit, (float)c->config.outputLimit);

    SpeedControl_Reset(c);
    c->enabled = false;
    c->config.setOutput(0, 0);
    return true;
}

void SpeedControl_Reset(SpeedControl_Controller *c)
{
    if (c == 0) {
        return;
    }

    PID_Reset(&c->leftPid);
    PID_Reset(&c->rightPid);

    /* 用当前计数初始化基准，避免第一拍把开机以来的全部计数当成一个周期的位移。 */
    c->lastLeftCount = (c->config.readLeftEncoder != 0) ? c->config.readLeftEncoder() : 0;
    c->lastRightCount = (c->config.readRightEncoder != 0) ? c->config.readRightEncoder() : 0;
    c->leftDelta = 0;
    c->rightDelta = 0;

    c->targetLeftCmps = 0.0f;
    c->targetRightCmps = 0.0f;
    c->leftSpeedCmps = 0.0f;
    c->rightSpeedCmps = 0.0f;
    c->carSpeedCmps = 0.0f;
    c->feedbackInitialized = false;

    c->leftDistanceCm = 0.0f;
    c->rightDistanceCm = 0.0f;
    c->carDistanceCm = 0.0f;

    c->leftOutput = 0;
    c->rightOutput = 0;

    c->fault = SPEED_CONTROL_FAULT_NONE;
    c->leftDirectionFaultCount = 0U;
    c->rightDirectionFaultCount = 0U;
    c->leftDirectionGraceCount = 0U;
    c->rightDirectionGraceCount = 0U;
    c->overspeedFaultCount = 0U;
}

void SpeedControl_Enable(SpeedControl_Controller *c, bool enabled)
{
    if (c == 0) {
        return;
    }

    if (enabled && !c->enabled) {
        /* 使能瞬间必须清掉停车期间残留的积分，否则会窜车。 */
        SpeedControl_Reset(c);
    }
    c->enabled = enabled;
    if (!enabled) {
        c->config.setOutput(0, 0);
        c->leftOutput = 0;
        c->rightOutput = 0;
    }
}

void SpeedControl_SetTargetCmps(SpeedControl_Controller *c, float leftCmps, float rightCmps)
{
    if (c == 0) {
        return;
    }

    /* 目标换向时给一段宽限期，避免滤波反馈滞后被误判为接线反。 */
    if ((c->targetLeftCmps * leftCmps) < 0.0f) {
        c->leftDirectionGraceCount = c->config.directionFaultConfirmCycles;
        c->leftDirectionFaultCount = 0U;
    }
    if ((c->targetRightCmps * rightCmps) < 0.0f) {
        c->rightDirectionGraceCount = c->config.directionFaultConfirmCycles;
        c->rightDirectionFaultCount = 0U;
    }

    c->targetLeftCmps = leftCmps;
    c->targetRightCmps = rightCmps;
}

void SpeedControl_SetDifferential(SpeedControl_Controller *c, float linearCmps, float angularCmps)
{
    if (c == 0) {
        return;
    }
    SpeedControl_SetTargetCmps(c, linearCmps - angularCmps, linearCmps + angularCmps);
}

bool SpeedControl_SetPidGains(SpeedControl_Controller *c,
                              float leftKp, float leftKi, float leftKd,
                              float rightKp, float rightKi, float rightKd)
{
    if (c == 0) {
        return false;
    }

    c->config.leftKp = leftKp;
    c->config.leftKi = leftKi;
    c->config.leftKd = leftKd;
    c->config.rightKp = rightKp;
    c->config.rightKi = rightKi;
    c->config.rightKd = rightKd;
    PID_SetGains(&c->leftPid, leftKp, leftKi, leftKd);
    PID_SetGains(&c->rightPid, rightKp, rightKi, rightKd);
    return true;
}

bool SpeedControl_SetCompensation(SpeedControl_Controller *c,
                                  float leftFeedforward, float rightFeedforward,
                                  int16_t leftDeadzone, int16_t rightDeadzone)
{
    if (c == 0) {
        return false;
    }

    c->config.leftFeedforwardPwmPerCmps = leftFeedforward;
    c->config.rightFeedforwardPwmPerCmps = rightFeedforward;
    c->config.leftDeadzonePwm = leftDeadzone;
    c->config.rightDeadzonePwm = rightDeadzone;
    return true;
}

static void SpeedControl_LatchFault(SpeedControl_Controller *c, SpeedControl_Fault fault)
{
    c->fault = fault;
    c->leftOutput = 0;
    c->rightOutput = 0;
    c->config.setOutput(0, 0);
    PID_Reset(&c->leftPid);
    PID_Reset(&c->rightPid);
}

/* 检查一侧的方向故障；返回 true 表示已确认故障。 */
static bool SpeedControl_CheckDirection(const SpeedControl_Controller *c,
                                        float target, float feedback,
                                        uint8_t *faultCount, uint8_t *graceCount)
{
    if (*graceCount > 0U) {
        (*graceCount)--;
        *faultCount = 0U;
        return false;
    }

    /* 低速段反馈量化噪声大，不做方向判定，只在明确的中高速冲突时报错。 */
    if ((CHESSCAR_ABS(target) < c->config.directionFaultMinTargetCmps) ||
        (CHESSCAR_ABS(feedback) < c->config.directionFaultMinFeedbackCmps)) {
        *faultCount = 0U;
        return false;
    }

    if ((target * feedback) < 0.0f) {
        (*faultCount)++;
        return (*faultCount >= c->config.directionFaultConfirmCycles);
    }

    *faultCount = 0U;
    return false;
}

void SpeedControl_Update(SpeedControl_Controller *c)
{
    int32_t leftCount, rightCount;
    float leftRaw, rightRaw;
    float leftPidOut, rightPidOut;
    int32_t leftFinal, rightFinal;

    if (c == 0) {
        return;
    }

    /* ---------- 1. 读累计计数并差分，得到本周期位移 ---------- */
    leftCount = c->config.readLeftEncoder() * c->config.leftEncoderSign;
    rightCount = c->config.readRightEncoder() * c->config.rightEncoderSign;
    c->leftDelta = leftCount - c->lastLeftCount;
    c->rightDelta = rightCount - c->lastRightCount;
    c->lastLeftCount = leftCount;
    c->lastRightCount = rightCount;

    /* ---------- 2. 换算速度并低通滤波 ---------- */
    leftRaw = Encoder_CountToSpeedCmps(c->leftDelta, &c->config.encoder);
    rightRaw = Encoder_CountToSpeedCmps(c->rightDelta, &c->config.encoder);
    if (!c->feedbackInitialized) {
        c->leftSpeedCmps = leftRaw;
        c->rightSpeedCmps = rightRaw;
        c->feedbackInitialized = true;
    } else {
        c->leftSpeedCmps = ChessCar_LowPass(c->leftSpeedCmps, leftRaw, c->config.feedbackFilterK);
        c->rightSpeedCmps = ChessCar_LowPass(c->rightSpeedCmps, rightRaw, c->config.feedbackFilterK);
    }
    c->carSpeedCmps = (c->leftSpeedCmps + c->rightSpeedCmps) * 0.5f;

    /* ---------- 3. 里程累计 ---------- */
    c->leftDistanceCm += Encoder_CountToDistanceCm(c->leftDelta, &c->config.encoder);
    c->rightDistanceCm += Encoder_CountToDistanceCm(c->rightDelta, &c->config.encoder);
    c->carDistanceCm = (c->leftDistanceCm + c->rightDistanceCm) * 0.5f;

    /* ---------- 4. 故障已锁定或未使能，保持零输出 ---------- */
    if ((c->fault != SPEED_CONTROL_FAULT_NONE) || !c->enabled) {
        c->leftOutput = 0;
        c->rightOutput = 0;
        c->config.setOutput(0, 0);
        return;
    }

    /* ---------- 5. 安全自检 ---------- */
    if ((CHESSCAR_ABS(c->leftSpeedCmps) > c->config.overspeedLimitCmps) ||
        (CHESSCAR_ABS(c->rightSpeedCmps) > c->config.overspeedLimitCmps)) {
        c->overspeedFaultCount++;
        if (c->overspeedFaultCount >= c->config.overspeedConfirmCycles) {
            SpeedControl_LatchFault(c, SPEED_CONTROL_FAULT_OVERSPEED);
            return;
        }
    } else {
        c->overspeedFaultCount = 0U;
    }

    if (SpeedControl_CheckDirection(c, c->targetLeftCmps, c->leftSpeedCmps,
                                   &c->leftDirectionFaultCount, &c->leftDirectionGraceCount) ||
        SpeedControl_CheckDirection(c, c->targetRightCmps, c->rightSpeedCmps,
                                   &c->rightDirectionFaultCount, &c->rightDirectionGraceCount)) {
        SpeedControl_LatchFault(c, SPEED_CONTROL_FAULT_ENCODER_DIRECTION);
        return;
    }

    /* ---------- 6. 前馈 + PID ---------- */
    leftPidOut = PID_RunPosition(&c->leftPid, c->targetLeftCmps, c->leftSpeedCmps);
    rightPidOut = PID_RunPosition(&c->rightPid, c->targetRightCmps, c->rightSpeedCmps);

    leftFinal = (int32_t)(leftPidOut +
                          c->config.leftFeedforwardPwmPerCmps * c->targetLeftCmps);
    rightFinal = (int32_t)(rightPidOut +
                           c->config.rightFeedforwardPwmPerCmps * c->targetRightCmps);

    /* ---------- 7. 死区补偿：目标为 0 时必须真正停住，不要补偿 ---------- */
    if (c->targetLeftCmps != 0.0f) {
        leftFinal = ChessCar_ApplyDeadzone(leftFinal, c->config.leftDeadzonePwm,
                                           c->config.outputLimit);
    } else {
        leftFinal = 0;
    }
    if (c->targetRightCmps != 0.0f) {
        rightFinal = ChessCar_ApplyDeadzone(rightFinal, c->config.rightDeadzonePwm,
                                            c->config.outputLimit);
    } else {
        rightFinal = 0;
    }

    /* ---------- 8. 方向符号 + 限幅 + 输出 ---------- */
    leftFinal *= c->config.leftMotorSign;
    rightFinal *= c->config.rightMotorSign;
    c->leftOutput = (int16_t)ChessCar_ClampInt32(leftFinal,
                                                 -c->config.outputLimit,
                                                 c->config.outputLimit);
    c->rightOutput = (int16_t)ChessCar_ClampInt32(rightFinal,
                                                  -c->config.outputLimit,
                                                  c->config.outputLimit);
    c->config.setOutput(c->leftOutput, c->rightOutput);
}

void SpeedControl_Stop(SpeedControl_Controller *c)
{
    if (c == 0) {
        return;
    }

    c->targetLeftCmps = 0.0f;
    c->targetRightCmps = 0.0f;
    c->leftOutput = 0;
    c->rightOutput = 0;
    PID_Reset(&c->leftPid);
    PID_Reset(&c->rightPid);
    c->config.setOutput(0, 0);
}

void SpeedControl_ClearFault(SpeedControl_Controller *c)
{
    if (c == 0) {
        return;
    }
    c->fault = SPEED_CONTROL_FAULT_NONE;
    c->leftDirectionFaultCount = 0U;
    c->rightDirectionFaultCount = 0U;
    c->overspeedFaultCount = 0U;
    PID_Reset(&c->leftPid);
    PID_Reset(&c->rightPid);
}

bool SpeedControl_HasFault(const SpeedControl_Controller *c)
{
    return (c != 0) && (c->fault != SPEED_CONTROL_FAULT_NONE);
}

SpeedControl_Fault SpeedControl_GetFault(const SpeedControl_Controller *c)
{
    return (c != 0) ? c->fault : SPEED_CONTROL_FAULT_NONE;
}

const char *SpeedControl_FaultName(SpeedControl_Fault fault)
{
    switch (fault) {
        case SPEED_CONTROL_FAULT_NONE:              return "NONE";
        case SPEED_CONTROL_FAULT_ENCODER_DIRECTION: return "ENC_DIR";
        case SPEED_CONTROL_FAULT_OVERSPEED:         return "OVERSPEED";
        default:                                    return "UNKNOWN";
    }
}

float SpeedControl_GetLeftSpeedCmps(const SpeedControl_Controller *c)
{
    return (c != 0) ? c->leftSpeedCmps : 0.0f;
}
float SpeedControl_GetRightSpeedCmps(const SpeedControl_Controller *c)
{
    return (c != 0) ? c->rightSpeedCmps : 0.0f;
}
float SpeedControl_GetCarSpeedCmps(const SpeedControl_Controller *c)
{
    return (c != 0) ? c->carSpeedCmps : 0.0f;
}
float SpeedControl_GetCarDistanceCm(const SpeedControl_Controller *c)
{
    return (c != 0) ? c->carDistanceCm : 0.0f;
}
int16_t SpeedControl_GetLeftOutput(const SpeedControl_Controller *c)
{
    return (c != 0) ? c->leftOutput : 0;
}
int16_t SpeedControl_GetRightOutput(const SpeedControl_Controller *c)
{
    return (c != 0) ? c->rightOutput : 0;
}
