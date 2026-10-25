#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pid_controller.h"
#include "encoder_math.h"
#include "line_sensor.h"
#include "line_follow.h"
#include "link_protocol.h"
#include "car_physical_config.h"

static int fails=0;
#define CHECK(c,msg) do{ if(!(c)){printf("FAIL: %s\n",msg);fails++;} else printf("ok  : %s\n",msg);}while(0)

/* ---- 协议回环 ---- */
static uint8_t txbuf[64]; static uint8_t txlen;
static int rxCount; static LinkProtocol_Frame rxFrame;
static bool mysend(const uint8_t*d,uint8_t n){ memcpy(txbuf,d,n); txlen=n; return true; }
static void myrecv(const LinkProtocol_Frame*f){ rxCount++; rxFrame=*f; }

int main(void){
    /* ===== 1. 编码器换算 ===== */
    Encoder_Config ec = { CAR_WHEEL_DIAMETER_CM, CAR_GEAR_RATIO,
                          CAR_ENCODER_COUNTS_PER_MOTOR_REV,
                          (float)CAR_SPEED_LOOP_PERIOD_MS };
    float cpr = CAR_ENCODER_COUNTS_PER_WHEEL_REV;
    printf("counts per wheel rev = %.2f\n", cpr);
    CHECK(fabsf(cpr-1061.27f)<1.0f, "每圈计数约 1061");
    /* 转一圈应等于一个周长 */
    float d = Encoder_CountToDistanceCm((int32_t)cpr, &ec);
    printf("one rev distance = %.3f cm (circumference %.3f)\n", d, CAR_WHEEL_CIRCUMFERENCE_CM);
    CHECK(fabsf(d-CAR_WHEEL_CIRCUMFERENCE_CM)<0.05f, "一圈计数换算等于周长");
    /* 往返一致性 */
    int32_t c10 = Encoder_SpeedCmpsToCount(10.0f,&ec);
    float back = Encoder_CountToSpeedCmps(c10,&ec);
    printf("10cm/s -> %d counts/10ms -> %.3f cm/s\n", c10, back);
    CHECK(fabsf(back-10.0f)<0.5f, "速度<->计数往返一致");
    printf("speed quantum = %.4f cm/s\n", Encoder_SpeedQuantumCmps(&ec));
    CHECK(Encoder_CountToSpeedCmps(-c10,&ec)<0.0f, "倒车计数为负速度");
    /* 非法配置 */
    Encoder_Config bad = {0};
    CHECK(Encoder_CountToSpeedCmps(100,&bad)==0.0f, "非法配置返回0");

    /* ===== 2. PID 限幅与抗饱和 ===== */
    PID_Controller pid;
    PID_Init(&pid, 10.0f, 2.0f, 0.0f, 500.0f, 1000.0f);
    /* 持续大正偏差，输出必须被限幅且积分不应无限增长 */
    float out=0;
    for(int i=0;i<500;i++) out=PID_RunPosition(&pid,100.0f,0.0f);
    printf("saturated out=%.1f integral=%.1f\n", out, pid.integral);
    CHECK(out<=1000.0f+1e-3f, "位置式输出不超限幅");
    CHECK(pid.integral<=500.0f+1e-3f, "积分不超积分限幅");
    /* 反向偏差应能让积分退出饱和 */
    for(int i=0;i<500;i++) out=PID_RunPosition(&pid,-100.0f,0.0f);
    printf("reverse out=%.1f integral=%.1f\n", out, pid.integral);
    CHECK(out>=-1000.0f-1e-3f && out<0.0f, "反向后输出反号且不超限");
    /* 收敛性：模拟一阶惯性被控对象 */
    /*
     * 一阶惯性模型：稳态 plant = u * 0.1，故目标 50 需要 u=500。
     * 注意 integralLimit 必须 >= 消除静差所需的积分量（这里约 450），
     * 否则限幅本身就会留下静差——这是实车调参最常见的误判点：
     * 以为 Ki 不够，其实是积分限幅卡住了。
     */
    PID_Init(&pid, 8.0f, 1.5f, 0.0f, 1000.0f, 1000.0f);
    float plant=0.0f, target=50.0f;
    for(int i=0;i<600;i++){ float u=PID_RunPosition(&pid,target,plant); plant += (u*0.1f - plant)*0.05f; }
    printf("converged plant=%.3f integral=%.1f (target %.1f)\n", plant, pid.integral, target);
    CHECK(fabsf(plant-target)<1.0f, "位置式PID能收敛到目标(积分消除静差)");

    /* 验证积分限幅确实是上面静差的成因，并确认抗饱和不会永久冻结积分。 */
    PID_Init(&pid, 8.0f, 1.5f, 0.0f, 400.0f, 1000.0f);
    plant=0.0f;
    for(int i=0;i<600;i++){ float u=PID_RunPosition(&pid,target,plant); plant += (u*0.1f - plant)*0.05f; }
    printf("limited integral=%.1f plant=%.3f\n", pid.integral, plant);
    CHECK(fabsf(pid.integral-400.0f)<1.0f, "积分被限幅在400导致残留静差");
    CHECK(plant<target, "积分限幅过小会留下静差(调参须知)");
    /* Reset 清状态 */
    PID_Reset(&pid);
    CHECK(pid.integral==0.0f && pid.error==0.0f, "Reset 清空运行状态");
    /* 增量式限幅 */
    PID_Init(&pid, 5.0f, 1.0f, 0.5f, 100.0f, 600.0f);
    for(int i=0;i<300;i++) out=PID_RunIncremental(&pid,80.0f,0.0f);
    printf("incremental out=%.1f\n", out);
    CHECK(out<=600.0f+1e-3f, "增量式输出不超限幅");
    /* 空指针安全 */
    CHECK(PID_RunPosition(0,1,1)==0.0f, "PID 空指针安全");

    /* ===== 3. 探头加权偏差 ===== */
    LineSensor_Config lc; LineSensor_State ls;
    LineSensor_LoadDefaultConfig(&lc);
    LineSensor_Init(&ls,&lc);
    LineSensor_UpdateMask(&ls, 0x18);            /* bit3,bit4 = 中间两颗 */
    printf("center mask err=%.3f hits=%u\n", ls.error, ls.hitCount);
    CHECK(fabsf(ls.error)<1e-6f, "正中压线偏差为0");
    LineSensor_UpdateMask(&ls, 0x01);            /* 最左 */
    printf("leftmost err=%.3f\n", ls.error);
    CHECK(ls.error<0.0f, "线在最左偏差为负");
    LineSensor_UpdateMask(&ls, 0x80);            /* 最右 */
    printf("rightmost err=%.3f\n", ls.error);
    CHECK(ls.error>0.0f, "线在最右偏差为正");
    float savedErr = ls.error;
    LineSensor_UpdateMask(&ls, 0x00);            /* 丢线 */
    printf("lost-line err=%.3f found=%d allWhite=%d\n", ls.error, ls.lineFound, ls.allWhite);
    CHECK(!ls.lineFound && ls.allWhite, "全白判定为丢线");
    CHECK(fabsf(ls.error-savedErr)<1e-6f, "丢线沿用上次有效偏差");
    LineSensor_UpdateMask(&ls, 0xFF);
    CHECK(ls.allBlack && fabsf(ls.error)<1e-6f, "全黑判定且偏差为0");
    /* 模拟量归一化 */
    uint16_t white[8]={600,600,600,600,600,600,600,600};
    uint16_t black[8]={3000,3000,3000,3000,3000,3000,3000,3000};
    LineSensor_Calibrate(&ls,white,false);
    LineSensor_Calibrate(&ls,black,true);
    uint16_t adc[8]={600,600,600,3000,3000,600,600,600};
    LineSensor_UpdateAnalog(&ls,adc);
    printf("analog mask=0x%02X err=%.3f norm[3]=%d norm[0]=%d\n",
           ls.mask, ls.error, ls.normalized[3], ls.normalized[0]);
    CHECK(ls.mask==0x18, "模拟量归一化生成正确掩码");
    CHECK(ls.normalized[3]==1000 && ls.normalized[0]==0, "归一化满量程正确");

    /* ===== 4. 循迹外环 ===== */
    LineFollow_Config fc; LineFollow_State fs;
    LineFollow_LoadDefaultConfig(&fc);
    LineFollow_Init(&fs,&fc);
    LineSensor_UpdateMask(&ls,0x80);             /* 线在右 */
    LineFollow_Update(&fs,&ls);
    printf("line right: L=%.2f R=%.2f corr=%.2f\n",
           fs.targetLeftCmps, fs.targetRightCmps, fs.correction);
    CHECK(fs.targetLeftCmps>fs.targetRightCmps, "线在右时左轮快于右轮(右转)");
    LineSensor_UpdateMask(&ls,0x01);             /* 线在左 */
    LineFollow_Update(&fs,&ls);
    printf("line left : L=%.2f R=%.2f\n", fs.targetLeftCmps, fs.targetRightCmps);
    CHECK(fs.targetLeftCmps<fs.targetRightCmps, "线在左时右轮快于左轮(左转)");
    CHECK(fs.targetLeftCmps>=fc.minTargetCmps && fs.targetRightCmps<=fc.maxTargetCmps,
          "外环目标在限幅范围内");
    /* 全黑应请求停车 */
    LineSensor_UpdateMask(&ls,0xFF);
    LineFollow_Update(&fs,&ls);
    CHECK(LineFollow_IsStopRequested(&fs), "全黑请求停车");
    /* 丢线超时停车 */
    LineFollow_Reset(&fs);
    LineSensor_UpdateMask(&ls,0x00);
    for(int i=0;i<fc.lostLineStopCycles+2;i++) LineFollow_Update(&fs,&ls);
    CHECK(LineFollow_IsStopRequested(&fs), "丢线超时后请求停车");

    /* ===== 5. 协议回环与健壮性 ===== */
    LinkProtocol_Context ctx;
    LinkProtocol_Init(&ctx,mysend,myrecv);
    uint8_t pl[4]={0xDE,0xAD,0xBE,0xEF};
    CHECK(LinkProtocol_SendFrame(&ctx,LINK_MSG_CAR_STATUS,7,0,pl,4), "发送帧成功");
    printf("frame len=%u: ",txlen);
    for(int i=0;i<txlen;i++) printf("%02X ",txbuf[i]);
    printf("\n");
    CHECK(txlen==12, "帧长 = 8 + payload4");
    CHECK(txbuf[0]==0xA5 && txbuf[1]==0x5A, "帧头正确");
    rxCount=0;
    LinkProtocol_ProcessBuffer(&ctx,txbuf,txlen);
    printf("rxCount=%d type=0x%02X seq=%u len=%u valid=%u\n",
           rxCount,rxFrame.type,rxFrame.sequence,rxFrame.length,ctx.validFrameCount);
    CHECK(rxCount==1, "回环收到一帧");
    CHECK(rxFrame.type==LINK_MSG_CAR_STATUS && rxFrame.sequence==7, "字段还原正确");
    CHECK(memcmp(rxFrame.payload,pl,4)==0, "payload 还原正确");
    /* CRC 错误必须被拒绝 */
    rxCount=0; txbuf[6]^=0xFF;
    LinkProtocol_ProcessBuffer(&ctx,txbuf,txlen);
    printf("after corrupt: rxCount=%d crcErr=%u\n", rxCount, ctx.crcErrorCount);
    CHECK(rxCount==0 && ctx.crcErrorCount==1, "CRC 错误帧被丢弃并计数");
    /* 噪声前缀 + 拆包重新同步 */
    txbuf[6]^=0xFF; rxCount=0;
    uint8_t noise[3]={0x11,0xA5,0x00};
    LinkProtocol_ProcessBuffer(&ctx,noise,3);
    for(int i=0;i<txlen;i++) LinkProtocol_ProcessByte(&ctx,txbuf[i]);  /* 逐字节喂 */
    printf("after noise+split: rxCount=%d\n", rxCount);
    CHECK(rxCount==1, "噪声后能重新同步并逐字节解析");
    /* 超长 length 被拒绝 */
    uint16_t le=ctx.lengthErrorCount;
    uint8_t badf[6]={0xA5,0x5A,0x10,0x00,0x00,0xFF};
    LinkProtocol_ProcessBuffer(&ctx,badf,6);
    CHECK(ctx.lengthErrorCount==le+1, "超长 payload 被拒绝并计数");
    /* 序号递增回绕 */
    LinkProtocol_Context c2; LinkProtocol_Init(&c2,mysend,myrecv);
    uint8_t s0=LinkProtocol_AllocateSequence(&c2);
    uint8_t s1=LinkProtocol_AllocateSequence(&c2);
    CHECK(s0==0&&s1==1, "序号从0递增");
    /* CRC16-CCITT-FALSE 标准向量 "123456789" = 0x29B1 */
    uint16_t crc=LinkProtocol_Crc16((const uint8_t*)"123456789",9);
    printf("CRC16-CCITT-FALSE(\"123456789\") = 0x%04X (expect 0x29B1)\n", crc);
    CHECK(crc==0x29B1, "CRC16 符合标准测试向量");

    printf("\n==== %s (fails=%d) ====\n", fails?"SOME TESTS FAILED":"ALL TESTS PASSED", fails);
    return fails?1:0;
}
