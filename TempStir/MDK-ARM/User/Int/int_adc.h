/*
 * int_adc.h
 * BSP 层 ADC1 封装：4 路模拟量（板载 NTC / 液体 NTC / 加热电流 / 24V 分压）
 * 的轮询式单通道读取。接管 CubeMX 的 4 通道扫描配置，改为单通道软件触发，
 * 以获得最可靠的逐通道结果。
 */
#ifndef INT_ADC_H
#define INT_ADC_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef enum
{
    INT_ADC_CH_BOARD_TEMP = 0,   /* PB0 / ADC1_IN8  板载热敏 */
    INT_ADC_CH_LIQUID_TEMP,      /* PB1 / ADC1_IN9  液体热敏 */
    INT_ADC_CH_HEAT_CURRENT,     /* PA4 / ADC1_IN4  加热电流 */
    INT_ADC_CH_SUPPLY_24V,       /* PC0 / ADC1_IN10 24V 分压 */
    INT_ADC_CH_COUNT
} IntAdcChannel_t;

/* 接管 ADC1 为单通道非连续轮询模式并校准；应在调度器启动前调用一次。 */
void Int_AdcInit(void);

/* 读取单个通道原始值（12bit），成功返回 0。 */
int Int_AdcReadSingle(IntAdcChannel_t ch, uint16_t *outRaw);

/* 依次读取全部 4 通道，填入 outRaw[INT_ADC_CH_COUNT]；任一失败整体返回 -1。 */
int Int_AdcReadAll(uint16_t outRaw[INT_ADC_CH_COUNT]);

#endif /* INT_ADC_H */
