/*
 * int_key.c
 * 4 路面板按键事件扫描实现（纯事件，无业务语义）。
 *
 * 状态机：每拍对每个键维护一个"按住拍数"计数
 *   按下 → 计数 +1；松开 → 计数清零
 *   计数 == INT_KEY_DEBOUNCE_N            → press（消抖成立）
 *   计数 >  INT_KEY_LONG_DELAY 且首次     → longOn（此后不再报 longOn）
 *   计数 >  INT_KEY_LONG_DELAY 且已长按    → repeat（每 INT_KEY_REPEAT_EVERY 拍一次）
 *   上一拍计数非 0 而本拍为 0              → release；若期间未升级长按 → 同时报 click
 *
 * 上层用法：press/click/longOn/repeat 各自决定怎么用；若某个键的长按被
 * 页面切换占用，收到 longOn 后忽略该键的 repeat 即可。
 */
#include "int_key.h"
#include "int_gpio.h"

#define INT_KEY_CNT_MAX       60000U   /* 计数饱和值，避免长时间按住溢出 */

static uint16_t int_key_cnt[INT_KEY_NUM];   /* 每个键已按住的拍数 */
static uint8_t  int_key_armed;              /* 按下已成立且未升级长按的键（用于判定 click） */

void Int_KeyInit(void)
{
    uint8_t i;

    for (i = 0U; i < INT_KEY_NUM; i++) {
        int_key_cnt[i] = 0U;
    }
    int_key_armed = 0U;
}

void Int_KeyScan(Int_KeyEvent_t *ev)
{
    uint8_t  raw;
    uint8_t  bit;
    uint8_t  i;
    uint16_t cnt;
    uint32_t phase;

    if (ev == 0) {
        return;
    }

    ev->press   = 0U;
    ev->click   = 0U;
    ev->longOn  = 0U;
    ev->repeat  = 0U;
    ev->release = 0U;

    raw = Int_GpioReadKeys();
    ev->held = raw;

    for (i = 0U; i < INT_KEY_NUM; i++) {
        bit = (uint8_t)(1U << i);

        if ((raw & bit) != 0U) {
            if (int_key_cnt[i] < INT_KEY_CNT_MAX) {
                int_key_cnt[i]++;
            }
            cnt = int_key_cnt[i];

            if (cnt == INT_KEY_DEBOUNCE_N) {
                ev->press |= bit;
                int_key_armed |= bit;               /* 等释放时判定是"点击"还是"长按" */
            }

            if (cnt > INT_KEY_LONG_DELAY) {
                if ((int_key_armed & bit) != 0U) {
                    int_key_armed &= (uint8_t)(~bit);
                    ev->longOn |= bit;              /* 首次越过阈值：只报一次 */
                } else {
                    phase = (uint32_t)(cnt - INT_KEY_LONG_DELAY);
                    if ((phase % INT_KEY_REPEAT_EVERY) == 0U) {
                        ev->repeat |= bit;
                    }
                }
            }
        } else {
            if (int_key_cnt[i] != 0U) {             /* 上一拍还按着 → 本拍是释放 */
                ev->release |= bit;
                if ((int_key_armed & bit) != 0U) {
                    int_key_armed &= (uint8_t)(~bit);
                    ev->click |= bit;               /* 未升级长按就松开 = 短按点击 */
                }
            }
            int_key_cnt[i] = 0U;
        }
    }
}
