/*
 * int_key.h
 * BSP 层 4 路面板按键事件扫描：只负责消抖、长按判定与连发节奏，
 * 输出"事件"位图，不含任何业务语义（不用知道电机/页面/参数）。
 *
 * 引脚与有效电平来自 int_bsp_pins.h（KEY1~KEY4 = PC6~PC9，低有效）。
 * 调用约定：由 UI 任务每 40ms 调用一次 Int_KeyScan()。
 *
 * 事件定义（每个位图 bit0=KEY1 … bit3=KEY4）：
 *   press   按下边沿：连续 3 拍（≈120ms）稳定按下后产生一次
 *   click   短按释放：按住后未经长按就松开时产生一次（"点一下"）
 *   longOn  长按成立：按住超过 15 拍（≈600ms）的瞬间产生一次
 *   repeat  长按连发：长按成立后每 2 拍（≈80ms）产生一次
 *   release 释放边沿：任何情况下松开都产生一次
 *   held    当前电平（不是事件），便于上层做"按住"相关判断
 */
#ifndef INT_KEY_H
#define INT_KEY_H

#include <stdint.h>

#define INT_KEY_NUM   4U

/* 按键位定义 */
#define INT_KEY1_BIT  0x01U
#define INT_KEY2_BIT  0x02U
#define INT_KEY3_BIT  0x04U
#define INT_KEY4_BIT  0x08U

/* 扫描参数（与 UI 任务 40ms 周期配套） */
#define INT_KEY_DEBOUNCE_N    3U    /* 连续 3 拍一致 → 按下成立（≈120ms） */
#define INT_KEY_LONG_DELAY    15U   /* 超过 15 拍 → 升级为长按（≈600ms） */
#define INT_KEY_REPEAT_EVERY  2U    /* 长按后每 2 拍连发一次（≈80ms） */

typedef struct
{
    uint8_t press;    /* 按下边沿（一次性） */
    uint8_t click;    /* 短按释放（一次性） */
    uint8_t longOn;   /* 长按成立瞬间（一次性） */
    uint8_t repeat;   /* 长按连发（可持续） */
    uint8_t release;  /* 释放边沿（一次性） */
    uint8_t held;     /* 当前按住的键（电平） */
} Int_KeyEvent_t;

/* 初始化：清空内部计数与状态（在创建 UI 任务前调用）。 */
void Int_KeyInit(void);

/* 扫描一次（40ms 周期），填充本次产生的事件与当前电平。 */
void Int_KeyScan(Int_KeyEvent_t *ev);

#endif /* INT_KEY_H */
