/*
 * int_adc.c
 * ADC1 板级封装。CubeMX 把 ADC1 配成 4 通道连续扫描，这里在 Int_AdcInit()
 * 中改为单通道、软件触发、非连续轮询模式，每次读取独立配置一个通道，
 * 避免连续扫描的 EOC 时序问题，结果稳定可靠。
 *
 * 多线程安全：用 FreeRTOS 互斥量串行化访问（调度器启动前自动跳过加锁，
 * 与 bsp_i2c1.c 的处理一致）。
 */
#include "int_adc.h"
#include "adc.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static SemaphoreHandle_t s_adcMutex = NULL;

/* 通道号映射（与 int_bsp_pins.h 的 ADC 引脚一致）。 */
static const uint32_t s_adcChannel[INT_ADC_CH_COUNT] = {
    ADC_CHANNEL_8,    /* PB0 板载 NTC */
    ADC_CHANNEL_9,    /* PB1 液体 NTC */
    ADC_CHANNEL_4,    /* PA4 加热电流 */
    ADC_CHANNEL_10    /* PC0 24V 分压 */
};

void Int_AdcInit(void)
{
    /* 接管 ADC1 为单通道、非连续轮询模式（覆盖 CubeMX 的扫描配置）。 */
    hadc1.Init.ScanConvMode       = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion    = 1;
    (void)HAL_ADC_Init(&hadc1);

    if (s_adcMutex == NULL)
    {
        s_adcMutex = xSemaphoreCreateMutex();
    }

    (void)HAL_ADCEx_Calibration_Start(&hadc1);
}

static void Int_AdcLock(void)
{
    if ((s_adcMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        (void)xSemaphoreTake(s_adcMutex, portMAX_DELAY);
    }
}

static void Int_AdcUnlock(void)
{
    if ((s_adcMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        (void)xSemaphoreGive(s_adcMutex);
    }
}

int Int_AdcReadSingle(IntAdcChannel_t ch, uint16_t *outRaw)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    HAL_StatusTypeDef status;

    if ((ch >= INT_ADC_CH_COUNT) || (outRaw == NULL))
    {
        return -1;
    }

    Int_AdcLock();

    sConfig.Channel      = s_adcChannel[ch];
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    status = HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    if (status == HAL_OK)
    {
        status = HAL_ADC_Start(&hadc1);
    }
    if (status == HAL_OK)
    {
        status = HAL_ADC_PollForConversion(&hadc1, 10);
    }
    if (status == HAL_OK)
    {
        *outRaw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    (void)HAL_ADC_Stop(&hadc1);
    Int_AdcUnlock();

    return (status == HAL_OK) ? 0 : -1;
}

int Int_AdcReadAll(uint16_t outRaw[INT_ADC_CH_COUNT])
{
    int ret = 0;
    IntAdcChannel_t ch;

    for (ch = INT_ADC_CH_BOARD_TEMP; ch < INT_ADC_CH_COUNT; ch++)
    {
        if (Int_AdcReadSingle(ch, &outRaw[ch]) != 0)
        {
            ret = -1;
        }
    }

    return ret;
}
