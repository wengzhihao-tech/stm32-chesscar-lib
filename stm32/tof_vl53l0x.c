/**
 * @file tof_vl53l0x.c
 * @brief 多路 ToF 管理实现（地址分配 + 读数过滤）。
 *
 * ⚠ 说明：本文件实现的是**多传感器组织与过滤逻辑**，
 * 以及 VL53L0X 的基础寄存器级读距。
 * 完整的校准、SPAD 管理、长距模式配置请使用 ST 官方 ULD 驱动，
 * 把 Tof_ReadRangeMm() 替换为官方 API 调用即可。
 */
#include "tof_vl53l0x.h"

#include "chesscar_common.h"

#define TOF_DEFAULT_ADDRESS_7BIT   (0x29U)
#define TOF_REG_SLAVE_ADDRESS      (0x8AU) /**< I2C 从地址配置寄存器。 */
#define TOF_REG_RESULT_RANGE_MM    (0x1EU) /**< 测距结果高字节（需按型号确认）。 */
#define TOF_REG_SYSRANGE_START     (0x00U)
#define TOF_REG_RESULT_STATUS      (0x14U)
#define TOF_I2C_TIMEOUT_MS         (10U)
#define TOF_INVALID_DISTANCE_CM    (9999.0f)

static bool Tof_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = value;
    return HAL_I2C_Master_Transmit(&BSP_OLED_I2C, (uint16_t)(addr7 << 1),
                                   buf, 2U, TOF_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool Tof_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *out, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(&BSP_OLED_I2C, (uint16_t)(addr7 << 1),
                                &reg, 1U, TOF_I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    return HAL_I2C_Master_Receive(&BSP_OLED_I2C, (uint16_t)(addr7 << 1),
                                  out, len, TOF_I2C_TIMEOUT_MS) == HAL_OK;
}

static void Tof_SetXshut(const Tof_SensorConfig *cfg, bool high)
{
    if (cfg->xshutPort == 0) {
        return;
    }
    HAL_GPIO_WritePin(cfg->xshutPort, cfg->xshutPin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t Tof_Init(Tof_Manager *mgr)
{
    uint8_t i;
    uint8_t okCount = 0U;

    if (mgr == 0) {
        return 0U;
    }

    mgr->minValidCm = 4.5f;   /* 低于此值读数不可信（VL53L1X 最小 4cm） */
    mgr->maxValidCm = 130.0f; /* 短距模式实际可用上限约 1.3m */
    mgr->filterK = 0.5f;

    for (i = 0U; i < TOF_SENSOR_MAX_COUNT; ++i) {
        mgr->rawMm[i] = 0U;
        mgr->filteredCm[i] = TOF_INVALID_DISTANCE_CM;
        mgr->valid[i] = false;
        mgr->errorCount[i] = 0U;
    }

    /*
     * 步骤 1：全部关断。
     * 这一步不能省——若有模块仍在 0x29 上响应，
     * 后续改址会同时命中多个模块，导致地址全部相同且无法区分。
     */
    for (i = 0U; i < TOF_SENSOR_MAX_COUNT; ++i) {
        Tof_SetXshut(&mgr->sensors[i], false);
    }
    HAL_Delay(10U);

    /* 步骤 2~4：逐个唤醒并改址。 */
    for (i = 0U; i < TOF_SENSOR_MAX_COUNT; ++i) {
        Tof_SensorConfig *cfg = &mgr->sensors[i];

        if (cfg->xshutPort == 0) {
            cfg->present = false;
            continue;
        }

        Tof_SetXshut(cfg, true);
        HAL_Delay(5U); /* 等模块启动，太短会通信失败 */

        /*
         * 用默认地址 0x29 把它改到目标地址。
         * 寄存器 0x8A 写入的是 7 位地址值。
         * 注意：改址后断电不保留，每次上电都要重跑本流程。
         */
        if (Tof_WriteReg(TOF_DEFAULT_ADDRESS_7BIT, TOF_REG_SLAVE_ADDRESS,
                         cfg->assignedAddress)) {
            HAL_Delay(2U);
            cfg->present = true;
            okCount++;
        } else {
            cfg->present = false;
        }
    }

    return okCount;
}

/* 读取一次距离。返回 false 表示本次测量无效。 */
static bool Tof_ReadRangeMm(const Tof_SensorConfig *cfg, uint16_t *outMm)
{
    uint8_t buf[2];

    if (!cfg->present) {
        return false;
    }

    /* 触发单次测量。 */
    if (!Tof_WriteReg(cfg->assignedAddress, TOF_REG_SYSRANGE_START, 0x01U)) {
        return false;
    }

    /*
     * 这里不做阻塞等待完成。单次测量典型耗时 20~30ms，
     * 在 20ms 控制周期里阻塞等待会直接破坏控制节拍。
     * 正确做法是本拍触发、下一拍读取上一次的结果，
     * 相当于容忍一拍的延迟换取不阻塞。
     */
    if (!Tof_ReadRegs(cfg->assignedAddress, TOF_REG_RESULT_RANGE_MM, buf, 2U)) {
        return false;
    }

    *outMm = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return true;
}

void Tof_UpdateAll(Tof_Manager *mgr)
{
    uint8_t i;

    if (mgr == 0) {
        return;
    }

    for (i = 0U; i < TOF_SENSOR_MAX_COUNT; ++i) {
        uint16_t mm = 0U;
        float cm;

        if (!Tof_ReadRangeMm(&mgr->sensors[i], &mm)) {
            mgr->valid[i] = false;
            mgr->errorCount[i]++;
            continue;
        }

        mgr->rawMm[i] = mm;
        cm = (float)mm * 0.1f;

        /*
         * 量程门限：
         * 低于下限说明贴得太近（读数不可信），
         * 高于上限说明没打到东西或超出短距模式能力。
         * 两种情况都不能拿去做位姿校正。
         */
        if ((cm < mgr->minValidCm) || (cm > mgr->maxValidCm)) {
            mgr->valid[i] = false;
            continue;
        }

        /* 低通滤波：ToF 单次读数有 ±2cm 级别的抖动。 */
        if (mgr->filteredCm[i] >= TOF_INVALID_DISTANCE_CM) {
            mgr->filteredCm[i] = cm; /* 首次直接赋值，避免从 9999 慢慢收敛 */
        } else {
            mgr->filteredCm[i] = ChessCar_LowPass(mgr->filteredCm[i], cm,
                                                  mgr->filterK);
        }
        mgr->valid[i] = true;
    }
}

float Tof_GetDistanceCm(const Tof_Manager *mgr, Tof_Position pos)
{
    if ((mgr == 0) || ((uint8_t)pos >= TOF_SENSOR_MAX_COUNT)) {
        return TOF_INVALID_DISTANCE_CM;
    }
    if (!mgr->valid[(uint8_t)pos]) {
        return TOF_INVALID_DISTANCE_CM; /* 上层按"无障碍"处理 */
    }
    return mgr->filteredCm[(uint8_t)pos];
}

bool Tof_IsValid(const Tof_Manager *mgr, Tof_Position pos)
{
    if ((mgr == 0) || ((uint8_t)pos >= TOF_SENSOR_MAX_COUNT)) {
        return false;
    }
    return mgr->valid[(uint8_t)pos];
}
