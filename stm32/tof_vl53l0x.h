/**
 * @file tof_vl53l0x.h
 * @brief 多路 VL53L0X/VL53L1X ToF 测距管理（I2C 地址冲突处理 + 挡板贴边校正输入）。
 *
 * 用途：本赛题没有引导线，车靠 ToF 测到四周 80mm 挡板来做绝对位置校正，
 * 同时用前向 ToF 检测对方车辆与障碍。
 *
 * ========== 必须知道的硬件约束（来自官方数据手册与第三方实测） ==========
 *
 * VL53L0X：
 *   - 默认 I2C 地址 **0x29**，所有模块出厂一样。
 *   - 多个模块挂同一条 I2C 必须改址，否则互相冲突完全读不出数据。
 *   - **改址后断电不保留**，每次上电都要重新执行改址流程。
 *   - 改址流程：先把所有 XSHUT 拉低（全部关断），再逐个拉高唤醒并立即改址。
 *   - 载板约 13×18×2mm，不含排针约 0.5g，典型工作电流 10mA（峰值可达 40mA）。
 *   - 数据手册未规定最小量程，第三方实测有效下限约 3cm，最大约 2m。
 *     来源：https://osoyoo.com/2019/04/21/arduino-lesson-vl53l0x-time-of-flight-distance-sensor
 *     多路改址方案参考：
 *     https://forum.seeedstudio.com/t/how-to-change-i2c-address-on-grove-time-of-flight-distance-sensor-vl53l0x/266866
 *
 * VL53L1X（更适合本赛题的升级选项）：
 *   - 最远 4m，测距频率最高 50Hz（短距模式），中/长距模式 30Hz。
 *   - **最小测距 4cm**，更近仍能检测到目标但数值不准。
 *   - 短距模式几乎不受环境光影响，但最大测距通常限于 1.3m。
 *   - 接收 FoV 可在 15°~27° 编程；暗光下 88% 白目标短距模式最大 130cm，
 *     测距误差约 ±20~±25mm。
 *   - 默认地址 0x52（8 位写地址形式），可编程。
 *     来源：https://www.st.com/resource/en/datasheet/vl53l1x.pdf
 *           https://www.pololu.com/product/3415
 *
 * ========== 对本赛题的直接影响 ==========
 *
 * 1. ±20~25mm 的测距误差意味着**单次测量不能直接当精确坐标**，
 *    本库在 PoseEstimator 里对校正做了姿态与量程门限，并建议多次测量取中值。
 * 2. 最小量程 3~4cm：车贴墙太近时读数不可信，所以贴边校正要在
 *    距墙 5~40cm 的区间内进行，不要贴死。
 * 3. FoV 15~27° 在 40cm 处的光斑直径约 10~19cm，**比棋子大得多**，
 *    所以 ToF **无法用来识别棋子**，只能测挡板和车辆这类大目标。
 *    棋子识别必须靠视觉。
 *
 * 本模块只做"读数 + 有效性过滤"，不含 ST 官方 API 的完整初始化序列。
 * 实际工程建议直接引入 ST 官方 VL53L0X/VL53L1X ULD 驱动，
 * 本文件负责把多路读数组织成位姿校正可用的形式。
 */
#ifndef TOF_VL53L0X_H
#define TOF_VL53L0X_H

#include "bsp_config.h"
#include <stdbool.h>
#include <stdint.h>

#define TOF_SENSOR_MAX_COUNT (4U)

/** 传感器安装方位。 */
typedef enum {
    TOF_POS_FRONT = 0,
    TOF_POS_LEFT,
    TOF_POS_RIGHT,
    TOF_POS_REAR
} Tof_Position;

typedef struct {
    GPIO_TypeDef *xshutPort; /**< XSHUT 控制脚，用于逐个唤醒改址。 */
    uint16_t xshutPin;
    uint8_t assignedAddress; /**< 改址后的 7 位地址，如 0x30/0x31/0x32/0x33。 */
    float mountOffsetCm;     /**< 传感器到车体中心沿测量方向的距离。 */
    bool present;            /**< 初始化是否成功。 */
} Tof_SensorConfig;

typedef struct {
    Tof_SensorConfig sensors[TOF_SENSOR_MAX_COUNT];
    uint16_t rawMm[TOF_SENSOR_MAX_COUNT];   /**< 最近一次原始读数 mm。 */
    float filteredCm[TOF_SENSOR_MAX_COUNT]; /**< 低通后的距离 cm。 */
    bool valid[TOF_SENSOR_MAX_COUNT];       /**< 本次读数是否可信。 */
    uint16_t errorCount[TOF_SENSOR_MAX_COUNT];
    float minValidCm;   /**< 有效量程下限，VL53L0X 建议 3.5，VL53L1X 建议 4.5。 */
    float maxValidCm;   /**< 有效量程上限。 */
    float filterK;      /**< 低通系数。 */
} Tof_Manager;

/**
 * @brief 初始化多路 ToF，含地址分配。
 *
 * **地址分配流程（必须严格按顺序，否则全部冲突）**：
 *   1. 把所有 XSHUT 拉低，让全部模块处于关断状态
 *   2. 逐个拉高一个模块的 XSHUT，等它启动
 *   3. 立即用默认地址 0x29 与它通信，把地址改成目标值
 *   4. 保持该模块上电，再处理下一个
 *
 * @return 成功初始化的传感器数量。
 */
uint8_t Tof_Init(Tof_Manager *mgr);

/** 读取所有传感器。建议 20~50ms 调用一次，不要快于传感器刷新率。 */
void Tof_UpdateAll(Tof_Manager *mgr);

/**
 * @brief 取某方位的距离（cm）。
 * @return 读数无效时返回一个大值（9999），便于上层按"无障碍"处理。
 */
float Tof_GetDistanceCm(const Tof_Manager *mgr, Tof_Position pos);

bool Tof_IsValid(const Tof_Manager *mgr, Tof_Position pos);

#endif /* TOF_VL53L0X_H */
