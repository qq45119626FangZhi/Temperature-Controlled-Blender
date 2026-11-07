/*
 * @file    app_tune.h
 * @brief   A2：现场实测整定支撑——按需打印一行统计快照（默认关闭）。
 *
 * 目的：A3 打滑门限 / A4 斜坡耗时 / A5 零偏温漂 / A6 振动门限 的实测整定，
 *       需要在实机上一边跑一边看「实测值 vs 门限」。本模块只做“读现成数据 + 打印”，
 *       不参与任何控制/判据，不改变既有行为。
 *
 * 使用：上位机写 0x003C（APP_MB_REG_TUNE_CTRL）
 *       bit0=1 打开周期打印（默认 0，保持旧版“运行期静默”）
 *       bit1=1 立即打印一行（自清零）
 *       bit2=1 清统计（自清零）
 * 打开后每 APP_TUNE_PERIOD_MS 打印一行 "[TUNE] ..."。
 */
#ifndef APP_TUNE_H
#define APP_TUNE_H

#include <stdint.h>

#define APP_TUNE_ENABLE        1U       /* 1 = 编译本模块（置 0 则 0x003C 写入无效果） */
#define APP_TUNE_PERIOD_MS     5000U    /* 周期打印间隔(ms) */

void     App_TuneInit(void);

/* 周期调用（CTRL 任务，dtMs = 200） */
void     App_TuneStep(uint32_t dtMs);

/* 0x003C 写入口：bit0 保存、bit1/bit2 立即执行。返回读回值（仅 bit0）。 */
uint16_t App_TuneSetCtrl(uint16_t bits);
uint16_t App_TuneGetCtrl(void);

/* 立即打印一行（内部使用，也可由外部触发） */
void     App_TunePrint(const char *why);

#endif /* APP_TUNE_H */
