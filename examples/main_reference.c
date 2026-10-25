/**
 * @file main_reference.c
 * @brief main.c 的参考写法（**不要直接替换 CubeMX 生成的 main.c**）。
 *
 * CubeMX 会覆盖 main.c 中 USER CODE 标记之外的所有内容，
 * 所以请只把下面标注的两段代码抄进对应的 USER CODE 区。
 */

/* ============================================================
 * 抄到 main.c 的 /* USER CODE BEGIN Includes *\/ 区：
 * ============================================================
 *
 *   #include "app_chesscar.h"
 *
 * ============================================================
 * 抄到 main.c 的 /* USER CODE BEGIN 2 *\/ 区（在所有
 * MX_xxx_Init() 之后、while(1) 之前）：
 * ============================================================
 *
 *   App_Init();
 *
 * ============================================================
 * 抄到 main.c 的 while (1) 循环体内 /* USER CODE BEGIN 3 *\/：
 * ============================================================
 *
 *   App_Loop();
 *
 * ============================================================
 * 编译配置提醒
 * ============================================================
 *
 * 1) Include Paths 需要加入：
 *      ../common
 *      ../stm32
 *      ../examples
 *
 * 2) 本工程大量使用 float。Cortex-M4（F407）请在工程里开启硬件 FPU：
 *      Keil: Target → Floating Point Hardware = Single Precision
 *      CubeIDE: C/C++ Build → Settings → MCU Settings → FPU = fpv4-sp-d16,
 *               Floating-point ABI = Hardware
 *    Cortex-M3（F103）没有 FPU，浮点靠软件模拟。
 *    实测 10ms 周期内两路 PID 的浮点运算量在 72MHz 下完全够用，
 *    但不要在 1ms 中断里加浮点运算。
 *
 * 3) 若使用 printf 调试，记得重定向 fputc 到串口，
 *    并注意 printf 在 115200 下单次可能阻塞数毫秒，不要放在控制周期里。
 */
