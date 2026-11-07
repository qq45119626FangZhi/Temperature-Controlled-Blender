/*
 * dri_ssd1315.c
 * SSD1315 OLED 驱动实现（128x64，I2C），指令集兼容 SSD1306。
 * I2C 访问统一经由 bsp_i2c1.c 的互斥封装，线程安全由 BSP 层与
 * "单 UI 任务持有显示权"的约定共同保证，本文件内不再额外加锁。
 */
#include "dri_ssd1315.h"
#include "int_bsp_i2c1.h"
#include <string.h>

/* 单次 I2C 事务超时时间 (ms)：100kHz 下传一页 129 字节约 12ms，余量充足 */
#define SSD1315_I2C_TIMEOUT     100U

/* I2C 控制字节：Co=0（后续还可跟数据），D/C# 区分命令/显存数据 */
#define SSD1315_CTRL_CMD        0x00U   /* D/C#=0：后续字节为命令 */
#define SSD1315_CTRL_DATA       0x40U   /* D/C#=1：后续字节为显存数据 */

/* 帧缓冲：每字节对应某列上连续的 8 个纵向像素（页寻址布局，bit0 在最上一行） */
static uint8_t s_frameBuffer[SSD1315_FB_SIZE];

/*
 * 影子缓冲：记录上一次真正写进屏幕显存的内容，用于"只刷新变化页"。
 * 多花 1KB RAM，换来的是：屏幕无变化时零 I2C 流量，只有变化页才上总线。
 * 上电时为静态零初始化，配合 s_shadowValid = 0 表示"影子内容不可信"，
 * 保证首帧一定整帧下发，不会因为比较结果恰好相同而漏刷、残留花屏。
 */
static uint8_t s_shadowBuffer[SSD1315_FB_SIZE];

/* 影子缓冲是否可信：0 = 不可信（下次刷屏必须整帧下发） */
static uint8_t s_shadowValid = 0U;

/* 刷屏暂存：1 字节控制字节 + 一页 128 字节显存数据 */
static uint8_t s_txBuf[1U + SSD1315_WIDTH];

/*
 * 发送单条命令。
 * cmd：SSD1315 命令字。
 * 返回：HAL_OK 成功；其余为 bsp_i2c1 透传的 HAL 错误码。
 */
static HAL_StatusTypeDef SSD1315_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2];

    buf[0] = SSD1315_CTRL_CMD;
    buf[1] = cmd;
    return Bsp_I2c1MasterTransmit(SSD1315_I2C_ADDR, buf, (uint16_t)sizeof(buf), SSD1315_I2C_TIMEOUT);
}

/*
 * 初始化 OLED。
 * 命令序列与 SSD1306 通用初始化流程一致（SSD1315 兼容），连续命令字节
 * 只需在开头跟一个 0x00 控制字节，可合并成单次 I2C 事务下发。
 */
HAL_StatusTypeDef SSD1315_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t initSeq[] =
    {
        SSD1315_CTRL_CMD,
        0xAEU,          /* 关显示，配置期间避免花屏 */
        0xD5U, 0x80U,   /* 显示时钟分频/振荡频率：默认 */
        0xA8U, 0x3FU,   /* 多路复用率：64（对应 64 行） */
        0xD3U, 0x00U,   /* 显示偏移：0 */
        0x40U,          /* 显示起始行：0 */
        0x8DU, 0x14U,   /* 电荷泵：使能（模块无外部 VCC 时必须开） */
        0x20U, 0x00U,   /* 寻址模式：水平寻址（刷屏时指针自动跨页递增） */
        0xA1U,          /* 段重映射：列 127 映射到 SEG0（左右镜像校正） */
        0xC8U,          /* COM 扫描方向：COM63 -> COM0（上下镜像校正） */
        0xDAU, 0x12U,   /* COM 硬件引脚配置：128x64 屏的常规值 */
        0x81U, 0xCFU,   /* 对比度：典型值 */
        0xD9U, 0xF1U,   /* 预充电周期：电荷泵供电时的推荐值 */
        0xDBU, 0x40U,   /* VCOMH 电平：默认 */
        0xA4U,          /* 显示内容跟随显存（忽略全亮测试模式） */
        0xA6U,          /* 正常显示（非反显） */
        0xAFU           /* 开显示 */
    };

    /* 创建 I2C1 互斥量。xSemaphoreCreateMutex 只申请堆内存，不依赖调度器，
       因此本函数安排在 vTaskStartScheduler() 之前调用是安全的。 */
    Bsp_I2c1Init();

    /* 探测 OLED 是否在线。若上次 MCU 异常复位时从机正占用 SDA，I2C1 会残留
       BUSY 导致探测失败，此时先按 I2C 规范打 9 个恢复时钟再探测一次。 */
    status = Bsp_I2c1IsDeviceReady(SSD1315_I2C_ADDR, 3U, SSD1315_I2C_TIMEOUT);
    if (status != HAL_OK)
    {
        (void)Bsp_I2c1RecoverBus();
        status = Bsp_I2c1IsDeviceReady(SSD1315_I2C_ADDR, 3U, SSD1315_I2C_TIMEOUT);
    }
    if (status != HAL_OK)
    {
        return status;  /* OLED 不在线，把错误码交给调用方决定降级策略 */
    }

    status = Bsp_I2c1MasterTransmit(SSD1315_I2C_ADDR, initSeq, (uint16_t)sizeof(initSeq), SSD1315_I2C_TIMEOUT);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 影子缓冲置为不可信：上电首帧必须整帧下发，避免显存残值造成上电花屏 */
    s_shadowValid = 0U;

    SSD1315_Clear();    /* 清缓冲并刷屏，避免显存残值造成上电花屏 */
    return HAL_OK;
}

/*
 * 刷屏：逐页与影子缓冲比对，**只把内容变化的页**写入显存。
 *
 * 每页流程：设定该页的显存窗口（列 0~127、页 page）-> 写 128 字节数据
 * -> 成功后把该页记入影子缓冲。
 * 好处：UI 层照旧"整帧重画"，而总线上只出现真正变化的页；
 *       屏幕静止时（例如只有 STATUS 不变）几乎不产生 I2C 流量，刷新可以做得很密。
 * 代价：每页多 6 字节窗口命令（相对 128 字节数据可忽略）与一次 memcmp。
 */
HAL_StatusTypeDef SSD1315_Refresh(void)
{
    HAL_StatusTypeDef status;
    uint8_t page;
    uint16_t offset;
    uint8_t window[] =
    {
        SSD1315_CTRL_CMD,
        0x21U, 0x00U, 0x7FU,    /* 列地址范围：0 ~ 127（整行） */
        0x22U, 0x00U, 0x00U     /* 页地址范围：本页 ~ 本页，逐页刷新 */
    };

    for (page = 0U; page < SSD1315_PAGES; page++)
    {
        offset = (uint16_t)page * SSD1315_WIDTH;

        /* 该页内容与屏上内容一致：跳过，不占用总线 */
        if ((s_shadowValid != 0U) &&
            (memcmp(&s_frameBuffer[offset], &s_shadowBuffer[offset], SSD1315_WIDTH) == 0))
        {
            continue;
        }

        window[5] = page;       /* 页地址起始 = 本页 */
        window[6] = page;       /* 页地址结束 = 本页 */

        status = Bsp_I2c1MasterTransmit(SSD1315_I2C_ADDR, window, (uint16_t)sizeof(window), SSD1315_I2C_TIMEOUT);
        if (status != HAL_OK)
        {
            return status;      /* 中途失败立即返回，未发送的页下次刷新会补发 */
        }

        s_txBuf[0] = SSD1315_CTRL_DATA;
        (void)memcpy(&s_txBuf[1], &s_frameBuffer[offset], SSD1315_WIDTH);
        status = Bsp_I2c1MasterTransmit(SSD1315_I2C_ADDR, s_txBuf, (uint16_t)sizeof(s_txBuf), SSD1315_I2C_TIMEOUT);
        if (status != HAL_OK)
        {
            return status;
        }

        /* 只有真正写成功才更新影子，保证失败的页下次会被重发 */
        (void)memcpy(&s_shadowBuffer[offset], &s_frameBuffer[offset], SSD1315_WIDTH);
    }

    /*
     * 整轮成功后才把影子标记为可信。
     * 中途若失败则直接 return，此时影子可能不完整：若原本就不可信（上电首帧），
     * 标记保持 0，下次刷新仍会整帧重发；若原本可信，失败的页因未更新影子而
     * 与帧缓冲不一致，下次刷新会自动补发该页。
     */
    s_shadowValid = 1U;
    return HAL_OK;
}

/* 强制整帧重刷：先让影子缓冲失效，再走一次刷新（所有页都会重发） */
HAL_StatusTypeDef SSD1315_RefreshAll(void)
{
    s_shadowValid = 0U;
    return SSD1315_Refresh();
}

/* 画点：只修改帧缓冲，对应 bit 置 1 为点亮、清 0 为熄灭 */
void SSD1315_DrawPixel(uint8_t x, uint8_t y, SSD1315_Color_t color)
{
    uint16_t index;

    if ((x >= SSD1315_WIDTH) || (y >= SSD1315_HEIGHT))
    {
        return;     /* 越界保护：忽略非法坐标 */
    }

    index = (uint16_t)x + ((uint16_t)(y / 8U) * SSD1315_WIDTH);
    if (color == SSD1315_COLOR_WHITE)
    {
        s_frameBuffer[index] |= (uint8_t)(1U << (y % 8U));
    }
    else
    {
        s_frameBuffer[index] &= (uint8_t)~(1U << (y % 8U));
    }
}

/*
 * 5x7 ASCII 字模表：列式存储，每个字符 5 字节，第 n 字节对应第 n 列，
 * bit0 为该列最上方像素、bit6 为最下方像素（bit7 未用）。
 * 覆盖可打印 ASCII 0x20(' ') ~ 0x7E('~')，共 95 个字形、475 字节常量。
 */
static const uint8_t s_font5x7[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, /* 0x20 空格 */
    0x00, 0x00, 0x5F, 0x00, 0x00, /* 0x21 !    */
    0x00, 0x07, 0x00, 0x07, 0x00, /* 0x22 "    */
    0x14, 0x7F, 0x14, 0x7F, 0x14, /* 0x23 #    */
    0x24, 0x2A, 0x7F, 0x2A, 0x12, /* 0x24 $    */
    0x23, 0x13, 0x08, 0x64, 0x62, /* 0x25 %    */
    0x36, 0x49, 0x55, 0x22, 0x50, /* 0x26 &    */
    0x00, 0x05, 0x03, 0x00, 0x00, /* 0x27 '    */
    0x00, 0x1C, 0x22, 0x41, 0x00, /* 0x28 (    */
    0x00, 0x41, 0x22, 0x1C, 0x00, /* 0x29 )    */
    0x08, 0x2A, 0x1C, 0x2A, 0x08, /* 0x2A *    */
    0x08, 0x08, 0x3E, 0x08, 0x08, /* 0x2B +    */
    0x00, 0x50, 0x30, 0x00, 0x00, /* 0x2C ,    */
    0x08, 0x08, 0x08, 0x08, 0x08, /* 0x2D -    */
    0x00, 0x60, 0x60, 0x00, 0x00, /* 0x2E .    */
    0x20, 0x10, 0x08, 0x04, 0x02, /* 0x2F /    */
    0x3E, 0x51, 0x49, 0x45, 0x3E, /* 0x30 0    */
    0x00, 0x42, 0x7F, 0x40, 0x00, /* 0x31 1    */
    0x42, 0x61, 0x51, 0x49, 0x46, /* 0x32 2    */
    0x21, 0x41, 0x45, 0x4B, 0x31, /* 0x33 3    */
    0x18, 0x14, 0x12, 0x7F, 0x10, /* 0x34 4    */
    0x27, 0x45, 0x45, 0x45, 0x39, /* 0x35 5    */
    0x3C, 0x4A, 0x49, 0x49, 0x30, /* 0x36 6    */
    0x01, 0x71, 0x09, 0x05, 0x03, /* 0x37 7    */
    0x36, 0x49, 0x49, 0x49, 0x36, /* 0x38 8    */
    0x06, 0x49, 0x49, 0x29, 0x1E, /* 0x39 9    */
    0x00, 0x36, 0x36, 0x00, 0x00, /* 0x3A :    */
    0x00, 0x56, 0x36, 0x00, 0x00, /* 0x3B ;    */
    0x00, 0x08, 0x14, 0x22, 0x41, /* 0x3C <    */
    0x14, 0x14, 0x14, 0x14, 0x14, /* 0x3D =    */
    0x41, 0x22, 0x14, 0x08, 0x00, /* 0x3E >    */
    0x02, 0x01, 0x51, 0x09, 0x06, /* 0x3F ?    */
    0x32, 0x49, 0x79, 0x41, 0x3E, /* 0x40 @    */
    0x7E, 0x11, 0x11, 0x11, 0x7E, /* 0x41 A    */
    0x7F, 0x49, 0x49, 0x49, 0x36, /* 0x42 B    */
    0x3E, 0x41, 0x41, 0x41, 0x22, /* 0x43 C    */
    0x7F, 0x41, 0x41, 0x22, 0x1C, /* 0x44 D    */
    0x7F, 0x49, 0x49, 0x49, 0x41, /* 0x45 E    */
    0x7F, 0x09, 0x09, 0x01, 0x01, /* 0x46 F    */
    0x3E, 0x41, 0x41, 0x51, 0x32, /* 0x47 G    */
    0x7F, 0x08, 0x08, 0x08, 0x7F, /* 0x48 H    */
    0x00, 0x41, 0x7F, 0x41, 0x00, /* 0x49 I    */
    0x20, 0x40, 0x41, 0x3F, 0x01, /* 0x4A J    */
    0x7F, 0x08, 0x14, 0x22, 0x41, /* 0x4B K    */
    0x7F, 0x40, 0x40, 0x40, 0x40, /* 0x4C L    */
    0x7F, 0x02, 0x04, 0x02, 0x7F, /* 0x4D M    */
    0x7F, 0x04, 0x08, 0x10, 0x7F, /* 0x4E N    */
    0x3E, 0x41, 0x41, 0x41, 0x3E, /* 0x4F O    */
    0x7F, 0x09, 0x09, 0x09, 0x06, /* 0x50 P    */
    0x3E, 0x41, 0x51, 0x21, 0x5E, /* 0x51 Q    */
    0x7F, 0x09, 0x19, 0x29, 0x46, /* 0x52 R    */
    0x46, 0x49, 0x49, 0x49, 0x31, /* 0x53 S    */
    0x01, 0x01, 0x7F, 0x01, 0x01, /* 0x54 T    */
    0x3F, 0x40, 0x40, 0x40, 0x3F, /* 0x55 U    */
    0x1F, 0x20, 0x40, 0x20, 0x1F, /* 0x56 V    */
    0x7F, 0x20, 0x18, 0x20, 0x7F, /* 0x57 W    */
    0x63, 0x14, 0x08, 0x14, 0x63, /* 0x58 X    */
    0x03, 0x04, 0x78, 0x04, 0x03, /* 0x59 Y    */
    0x61, 0x51, 0x49, 0x45, 0x43, /* 0x5A Z    */
    0x00, 0x7F, 0x41, 0x41, 0x00, /* 0x5B [    */
    0x02, 0x04, 0x08, 0x10, 0x20, /* 0x5C 反斜杠 */
    0x00, 0x41, 0x41, 0x7F, 0x00, /* 0x5D ]    */
    0x04, 0x02, 0x01, 0x02, 0x04, /* 0x5E ^    */
    0x40, 0x40, 0x40, 0x40, 0x40, /* 0x5F _    */
    0x00, 0x00, 0x00, 0x00, 0x00, /* 0x60 `    */
    0x20, 0x54, 0x54, 0x54, 0x78, /* 0x61 a    */
    0x7F, 0x48, 0x44, 0x44, 0x38, /* 0x62 b    */
    0x38, 0x44, 0x44, 0x44, 0x20, /* 0x63 c    */
    0x38, 0x44, 0x44, 0x48, 0x7F, /* 0x64 d    */
    0x38, 0x54, 0x54, 0x54, 0x18, /* 0x65 e    */
    0x08, 0x7E, 0x09, 0x01, 0x02, /* 0x66 f    */
    0x08, 0x14, 0x54, 0x54, 0x3C, /* 0x67 g    */
    0x7F, 0x08, 0x04, 0x04, 0x78, /* 0x68 h    */
    0x00, 0x44, 0x7D, 0x40, 0x00, /* 0x69 i    */
    0x20, 0x40, 0x44, 0x3D, 0x00, /* 0x6A j    */
    0x00, 0x7F, 0x10, 0x28, 0x44, /* 0x6B k    */
    0x00, 0x41, 0x7F, 0x40, 0x00, /* 0x6C l    */
    0x7C, 0x04, 0x18, 0x04, 0x78, /* 0x6D m    */
    0x7C, 0x08, 0x04, 0x04, 0x78, /* 0x6E n    */
    0x38, 0x44, 0x44, 0x44, 0x38, /* 0x6F o    */
    0x7C, 0x14, 0x14, 0x14, 0x08, /* 0x70 p    */
    0x08, 0x14, 0x14, 0x18, 0x7C, /* 0x71 q    */
    0x7C, 0x08, 0x04, 0x04, 0x08, /* 0x72 r    */
    0x48, 0x54, 0x54, 0x54, 0x20, /* 0x73 s    */
    0x04, 0x3F, 0x44, 0x40, 0x20, /* 0x74 t    */
    0x3C, 0x40, 0x40, 0x20, 0x7C, /* 0x75 u    */
    0x1C, 0x20, 0x40, 0x20, 0x1C, /* 0x76 v    */
    0x3C, 0x40, 0x30, 0x40, 0x3C, /* 0x77 w    */
    0x44, 0x28, 0x10, 0x28, 0x44, /* 0x78 x    */
    0x0C, 0x50, 0x50, 0x50, 0x3C, /* 0x79 y    */
    0x44, 0x64, 0x54, 0x4C, 0x44, /* 0x7A z    */
    0x00, 0x08, 0x36, 0x41, 0x00, /* 0x7B {    */
    0x00, 0x00, 0x7F, 0x00, 0x00, /* 0x7C |    */
    0x00, 0x41, 0x36, 0x08, 0x00, /* 0x7D }    */
    0x08, 0x08, 0x2A, 0x1C, 0x08  /* 0x7E ~    */
};

/* 字模首字符：可打印 ASCII 的起始值 */
#define SSD1315_FONT_FIRST_CHAR     0x20U

/*
 * 绘制单个字符：逐列取字模，逐行写像素。
 * 前景像素写 color，背景像素写反色，保证 5x7 区域内内容被整体覆盖。
 */
void SSD1315_DrawChar(uint8_t x, uint8_t y, char ch, SSD1315_Color_t color)
{
    uint8_t col;
    uint8_t row;
    uint8_t bits;
    uint16_t offset;
    SSD1315_Color_t bgColor;

    /* 越界或非可打印字符 -> 画成空格，避免取到非法字模数据 */
    if ((x + SSD1315_FONT_WIDTH > SSD1315_WIDTH) || (y + SSD1315_FONT_HEIGHT > SSD1315_HEIGHT))
    {
        return;
    }
    if ((ch < (char)SSD1315_FONT_FIRST_CHAR) || (ch > 0x7E))
    {
        ch = (char)SSD1315_FONT_FIRST_CHAR;
    }

    bgColor = (color == SSD1315_COLOR_WHITE) ? SSD1315_COLOR_BLACK : SSD1315_COLOR_WHITE;
    offset = (uint16_t)((uint8_t)ch - SSD1315_FONT_FIRST_CHAR) * SSD1315_FONT_WIDTH;

    for (col = 0U; col < SSD1315_FONT_WIDTH; col++)
    {
        bits = s_font5x7[offset + col];
        for (row = 0U; row < SSD1315_FONT_HEIGHT; row++)
        {
            if ((bits & (uint8_t)(1U << row)) != 0U)
            {
                SSD1315_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), color);
            }
            else
            {
                SSD1315_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), bgColor);
            }
        }
    }
}

/* 绘制字符串：字符步进 = 字宽 5 + 间隔 1；越界或遇 '\0' 停止 */
void SSD1315_DrawString(uint8_t x, uint8_t y, const char *str, SSD1315_Color_t color)
{
    uint8_t cursorX = x;

    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        if ((uint16_t)(cursorX + SSD1315_FONT_WIDTH) > SSD1315_WIDTH)
        {
            break;  /* 放不下下一个字符，停止绘制（不自动换行） */
        }
        SSD1315_DrawChar(cursorX, y, *str, color);
        cursorX = (uint8_t)(cursorX + SSD1315_FONT_WIDTH + 1U);
        str++;
    }
}

/* 整屏填充：白色填 0xFF（全亮），黑色填 0x00（全灭） */
void SSD1315_Fill(SSD1315_Color_t color)
{
    (void)memset(s_frameBuffer, (color == SSD1315_COLOR_WHITE) ? 0xFF : 0x00, SSD1315_FB_SIZE);
}

/* 清空帧缓冲：只改 RAM，不产生任何 I2C 流量 */
void SSD1315_ClearBuffer(void)
{
    SSD1315_Fill(SSD1315_COLOR_BLACK);
}

/*
 * 清屏：清空帧缓冲并立即刷屏到硬件（屏幕真的被擦黑）。
 * 与 SSD1315_ClearBuffer() 的区别就在"是否马上推到屏幕"：
 * 每帧重绘的 UI 必须用 ClearBuffer + 结尾统一 Refresh，否则每帧都会
 * 先把屏幕擦黑再重画 —— 视觉上是整屏闪烁，同时让差异页优化彻底失效。
 */
void SSD1315_Clear(void)
{
    SSD1315_ClearBuffer();
    (void)SSD1315_Refresh();
}

/* 设置对比度：0x81 后跟 1 字节亮度值 */
HAL_StatusTypeDef SSD1315_SetContrast(uint8_t contrast)
{
    HAL_StatusTypeDef status = SSD1315_WriteCmd(0x81U);

    if (status != HAL_OK)
    {
        return status;
    }
    return SSD1315_WriteCmd(contrast);
}

/* 开关显示：0xAF 开 / 0xAE 关（关显示不清显存） */
HAL_StatusTypeDef SSD1315_SetDisplayOn(uint8_t on)
{
    return SSD1315_WriteCmd((on != 0U) ? 0xAFU : 0xAEU);
}

/* 反显控制：0xA7 反显 / 0xA6 正常 */
HAL_StatusTypeDef SSD1315_SetInvert(uint8_t invert)
{
    return SSD1315_WriteCmd((invert != 0U) ? 0xA7U : 0xA6U);
}
