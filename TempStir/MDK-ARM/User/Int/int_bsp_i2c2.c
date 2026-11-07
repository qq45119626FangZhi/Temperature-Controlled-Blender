/*
 * int_bsp_i2c2.c
 * I2C2 的线程安全 HAL 适配层，服务于低温漂外部时钟（RTC）与 EEPROM。
 *
 * 两个器件共用一条物理总线（从机地址不同），HAL 的 hi2c2 状态机不可重入，
 * 业务模块只能调用本文件头文件暴露的接口，不直接操作 I2C HAL 状态机。
 * 调度器启动前没有任务并发，因此启动自检期间有意跳过互斥量。
 */
#include "int_bsp_i2c2.h"
#include "i2c.h"
#include "int_bsp_pins.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static SemaphoreHandle_t s_i2c2Mutex = NULL;

void Bsp_I2c2Init(void)
{
    if (s_i2c2Mutex == NULL)
    {
        s_i2c2Mutex = xSemaphoreCreateMutex();
    }
}

static void Bsp_I2c2BitDelay(void)
{
    volatile uint32_t count = SystemCoreClock / 1000000U;

    while (count > 0U)
    {
        count--;
        __NOP();
    }
}

HAL_StatusTypeDef Bsp_I2c2RecoverBus(void)
{
    GPIO_InitTypeDef gpio = {0};
    HAL_StatusTypeDef lineStatus;
    uint8_t pulse;

    /* 从设备占用 SDA 时复位 MCU，F1 I2C 可能残留 BUSY；先关闭并复位外设。 */
    (void)HAL_I2C_DeInit(&hi2c2);
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();

    HAL_GPIO_WritePin(EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin, GPIO_PIN_SET);
    gpio.Pin = EEPROM_SCL_Pin | EEPROM_SDA_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(EEPROM_SCL_GPIO_Port, &gpio);
    Bsp_I2c2BitDelay();

    /* 输出最多 9 个 SCL 脉冲，让从设备完成被中断的一个字节并释放 SDA。 */
    for (pulse = 0U; pulse < 9U; pulse++)
    {
        if (HAL_GPIO_ReadPin(EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin) == GPIO_PIN_SET)
        {
            break;
        }
        HAL_GPIO_WritePin(EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, GPIO_PIN_RESET);
        Bsp_I2c2BitDelay();
        HAL_GPIO_WritePin(EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, GPIO_PIN_SET);
        Bsp_I2c2BitDelay();
    }

    /* SCL 为高期间让 SDA 从低变高，生成符合 I2C 规范的 STOP。 */
    HAL_GPIO_WritePin(EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin, GPIO_PIN_RESET);
    Bsp_I2c2BitDelay();
    HAL_GPIO_WritePin(EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, GPIO_PIN_SET);
    Bsp_I2c2BitDelay();
    HAL_GPIO_WritePin(EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin, GPIO_PIN_SET);
    Bsp_I2c2BitDelay();

    lineStatus = ((HAL_GPIO_ReadPin(EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin) == GPIO_PIN_SET) &&(HAL_GPIO_ReadPin(EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin) == GPIO_PIN_SET)) ?HAL_OK : HAL_ERROR;

    /* MX_I2C2_Init() 将 PB10/PB11 恢复为复用开漏，并重新配置 100 kHz I2C2。 */
    MX_I2C2_Init();
    return lineStatus;
}

/* 仅在“总线被从机拉住”时恢复：判据是 I2C_FLAG_BUSY 卡死。
 * 普通 NACK（例如 EEPROM 页编程期间器件不应答）不触发恢复，
 * 否则会打断正在进行的内部写周期。返回 1 表示已执行恢复。 */
static uint8_t Bsp_I2c2RecoverIfBusy(void)
{
    if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) == RESET)
    {
        return 0U;
    }

    (void)Bsp_I2c2RecoverBus();
    return 1U;
}

static void Bsp_I2c2Lock(void)
{
    /* 启动阶段尚无任务并发；调度器启动后才获取互斥量。 */
    if ((s_i2c2Mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        (void)xSemaphoreTake(s_i2c2Mutex, portMAX_DELAY);
    }
}

static void Bsp_I2c2Unlock(void)
{
    /* 与 Bsp_I2c2Lock() 条件严格匹配，未获取时绝不释放。 */
    if ((s_i2c2Mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        (void)xSemaphoreGive(s_i2c2Mutex);
    }
}

HAL_StatusTypeDef Bsp_I2c2MasterTransmit(uint16_t devAddress, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    Bsp_I2c2Lock();
    status = HAL_I2C_Master_Transmit(&hi2c2, devAddress, (uint8_t *)data, size, timeout);
    if ((status != HAL_OK) && (Bsp_I2c2RecoverIfBusy() != 0U))
    {
        status = HAL_I2C_Master_Transmit(&hi2c2, devAddress, (uint8_t *)data, size, timeout);
    }
    Bsp_I2c2Unlock();

    return status;
}

HAL_StatusTypeDef Bsp_I2c2MemWrite(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,const uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    Bsp_I2c2Lock();
    status = HAL_I2C_Mem_Write(&hi2c2, devAddress, memAddress, memAddSize, (uint8_t *)data, size, timeout);
    if ((status != HAL_OK) && (Bsp_I2c2RecoverIfBusy() != 0U))
    {
        status = HAL_I2C_Mem_Write(&hi2c2, devAddress, memAddress, memAddSize, (uint8_t *)data, size, timeout);
    }
    Bsp_I2c2Unlock();

    return status;
}

HAL_StatusTypeDef Bsp_I2c2MemRead(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    Bsp_I2c2Lock();
    status = HAL_I2C_Mem_Read(&hi2c2, devAddress, memAddress, memAddSize, data, size, timeout);
    if ((status != HAL_OK) && (Bsp_I2c2RecoverIfBusy() != 0U))
    {
        status = HAL_I2C_Mem_Read(&hi2c2, devAddress, memAddress, memAddSize, data, size, timeout);
    }
    Bsp_I2c2Unlock();

    return status;
}

HAL_StatusTypeDef Bsp_I2c2IsDeviceReady(uint16_t devAddress, uint32_t trials, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    Bsp_I2c2Lock();
    status = HAL_I2C_IsDeviceReady(&hi2c2, devAddress, trials, timeout);
    if ((status != HAL_OK) && (Bsp_I2c2RecoverIfBusy() != 0U))
    {
        status = HAL_I2C_IsDeviceReady(&hi2c2, devAddress, trials, timeout);
    }
    Bsp_I2c2Unlock();

    return status;
}
