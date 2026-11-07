/*
 * dri_ssd1315.h
 * SSD1315 OLED 驱动（128x64 单色，I2C 接口），指令集兼容 SSD1306。
 *
 * 设计要点：
 *   1) I2C 访问统一走 bsp_i2c1.c 的互斥封装，本驱动不直接操作 HAL 的
 *      hi2c1，多任务环境下由 BSP 层完成事务级串行化；
 *   2) 片内维护 1KB 帧缓冲（128x64 / 8），画图函数只改缓冲，
 *      调用 SSD1315_Refresh() 才把整帧刷到屏幕上，避免边画边闪；
 *   3) 帧缓冲自身不带锁。请把显示权收敛到单个 UI 任务（与工程串口打印
 *      约定一致），否则多任务并发画图/刷屏会产生数据竞争；
 *   4) SSD1315_Init() 应在 vTaskStartScheduler() 之前调用：
 *      内部创建 I2C1 互斥量、按需做总线恢复，二者都只在启动阶段安全。
 */
#ifndef DRI_SSD1315_H
#define DRI_SSD1315_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== 屏体参数 ============================== */
#define SSD1315_WIDTH       128U                            /* 水平像素数 */
#define SSD1315_HEIGHT      64U                             /* 垂直像素数 */
#define SSD1315_PAGES       (SSD1315_HEIGHT / 8U)           /* 页数（每页 8 行像素） */
#define SSD1315_FB_SIZE     (SSD1315_WIDTH * SSD1315_PAGES) /* 帧缓冲字节数：1024 */

/*
 * I2C 从机地址（HAL 约定：7 位地址左移一位后的 8 位写地址）。
 * 市面 SSD1315 模块绝大多数为 7 位 0x3C -> 0x78；少数模块为 0x3D -> 0x7A，
 * 若总线上探测不到器件，按实际硬件改这一个宏即可。
 */
#define SSD1315_I2C_ADDR    0x78U

/* 画点颜色：单色屏只有亮/灭两种 */
typedef enum
{
    SSD1315_COLOR_BLACK = 0,    /* 熄灭像素 */
    SSD1315_COLOR_WHITE = 1     /* 点亮像素 */
} SSD1315_Color_t;

/*
 * 初始化：创建 I2C1 互斥量 -> 探测器件 -> （失败时先做总线恢复再探一次）
 *         -> 下发初始化命令序列 -> 清屏。
 * 返回 HAL_OK 表示 OLED 在线且初始化完成；其余为 HAL 错误码（多为器件不在线）。
 * 注意：必须在 vTaskStartScheduler() 之前调用，不得在运行期重复调用。
 */
HAL_StatusTypeDef SSD1315_Init(void);

/*
 * 清空帧缓冲：只把 1KB 帧缓冲填黑，**不碰 I2C、不刷屏**。
 *
 * 这是"每帧重建画面"型 UI 的正确起手式：先清缓冲再重画，最后统一调用
 * SSD1315_Refresh()。因为屏幕只收到最终帧与上一帧的差异页，所以
 * 既不会出现"擦黑→重画"的闪烁，也不会残留上一帧的字迹。
 */
void SSD1315_ClearBuffer(void);

/*
 * 清屏：清空帧缓冲并**立即刷屏到硬件**（真的把屏幕擦黑）。
 * 适用：上电初始化、确实需要整屏擦除的场合。
 * 警告：每帧重绘的 UI 不要用它，否则每帧都会先把屏幕擦黑再重画 ——
 *       表现为整屏闪烁/残影，并且让"只刷新变化页"的优化失效。
 */
void SSD1315_Clear(void);

/*
 * 刷屏：把帧缓冲刷新到显存（**只发变化的页**）。
 *
 * 内部维护一份 1KB 影子缓冲，记录"上次真正写进屏里的内容"：逐页比对，
 * 内容没变的页一个字节都不发，只有变化的页才走 I2C。
 * 因此 UI 层可以放心地"整帧重画"，实际总线流量只与真正变化的区域成正比
 * （I2C1 100kHz 下，整帧 8 页约 94ms；只变 1 页约 12ms，全屏无变化约 0ms）。
 *
 * 返回 HAL_OK 表示所有需要更新的页都写入成功；中途失败立即返回错误码，
 * 已成功写入的页会记入影子缓冲，下次刷新会自然补发失败的页。
 */
HAL_StatusTypeDef SSD1315_Refresh(void);

/*
 * 强制整帧重刷：把影子缓冲置为无效后调用 SSD1315_Refresh()。
 * 使用场景：怀疑屏幕内容与帧缓冲不一致（如总线干扰、外部复位屏幕）时，
 * 用它做一次全量对齐；上电初始化内部已自动整帧下发，无需手动调用。
 */
HAL_StatusTypeDef SSD1315_RefreshAll(void);

/*
 * 画点：在帧缓冲 (x, y) 处点亮/熄灭一个像素。
 * 坐标原点在左上角，x 向右 0~127，y 向下 0~63；越界坐标直接忽略。
 */
void SSD1315_DrawPixel(uint8_t x, uint8_t y, SSD1315_Color_t color);

/* ============== 字符绘制（5x7 字模，ASCII 0x20 ~ 0x7E） ============== */
#define SSD1315_FONT_WIDTH      5U      /* 字模宽度（像素） */
#define SSD1315_FONT_HEIGHT     7U      /* 字模高度（像素） */

/*
 * 绘制单个字符：在 (x, y) 处画一个 5x7 字符，占 5 列 7 行。
 * 字形区内的像素会被无条件改写（前景用 color，背景用反色），
 * 因此叠加绘制会盖掉原有内容，不会留下残影。
 * ch 超出 0x20~0x7E 范围时画成空格。
 */
void SSD1315_DrawChar(uint8_t x, uint8_t y, char ch, SSD1315_Color_t color);

/*
 * 绘制字符串：从 (x, y) 开始逐个画字符，字符间距 1 像素（步进 6 列）。
 * 遇到 '\0'、超出右边界或超出下边界即停止，不自动换行。
 */
void SSD1315_DrawString(uint8_t x, uint8_t y, const char *str, SSD1315_Color_t color);

/* 整屏填充：帧缓冲全部填为指定颜色（不刷屏，配合 SSD1315_Refresh 使用） */
void SSD1315_Fill(SSD1315_Color_t color);

/* 设置对比度：contrast 范围 0x00~0xFF，典型值 0xCF */
HAL_StatusTypeDef SSD1315_SetContrast(uint8_t contrast);

/* 开关显示：on 非 0 开显示（0xAF），为 0 关显示（0xAE，显存内容保留） */
HAL_StatusTypeDef SSD1315_SetDisplayOn(uint8_t on);

/* 反显控制：invert 非 0 反显（0xA7），为 0 正常显示（0xA6） */
HAL_StatusTypeDef SSD1315_SetInvert(uint8_t invert);

#ifdef __cplusplus
}
#endif

#endif /* DRI_SSD1315_H */
