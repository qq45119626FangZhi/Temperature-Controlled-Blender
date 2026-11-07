/*
 * int_spi.c
 * SPI2 板级封装。CS（LSM_NSS / PB12）由 GPIO 软件控制，电平取自 int_bsp_pins.h：
 *   LSM_CS_ACTIVE_STATE   = RESET（低有效，选中）
 *   LSM_CS_INACTIVE_STATE = SET  （释放）
 * LSM6DSM 读时序：首字节 = 地址 | 0x80，随后发送哑元并接收数据（全双工）。
 */
#include "int_spi.h"
#include "int_bsp_pins.h"

void Int_SpiInit(void)
{
    /* CS 引脚已在 gpio.c 配为 OUTPUT_PP 且上电置高（INACTIVE），这里再确保一次。 */
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, LSM_CS_INACTIVE_STATE);
}

static void Int_SpiCsAssert(void)
{
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, LSM_CS_ACTIVE_STATE);
}

static void Int_SpiCsRelease(void)
{
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, LSM_CS_INACTIVE_STATE);
}

uint8_t Int_SpiReadReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80U), 0x00U };
    uint8_t rx[2] = { 0U, 0U };

    Int_SpiCsAssert();
    (void)HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 100U);
    Int_SpiCsRelease();

    return rx[1];   /* 第 2 字节为寄存器值 */
}

void Int_SpiWriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), val };

    Int_SpiCsAssert();
    (void)HAL_SPI_Transmit(&hspi2, tx, 2U, 100U);
    Int_SpiCsRelease();
}

/* 突发读：首字节发 reg|0x80，随后 len 个哑元，回读第 1..len 字节。
 * 器件地址自增（IF_INC=1）保证一次 CS 内连续读到相邻寄存器。 */
uint8_t Int_SpiReadBuf(uint8_t reg, uint8_t *out, uint16_t len)
{
    uint8_t  tx[24];
    uint8_t  rx[24];
    uint16_t i;

    if ((out == NULL) || (len == 0U) || (len > 23U)) {
        return 0U;
    }

    tx[0] = (uint8_t)(reg | 0x80U);
    for (i = 1U; i <= len; i++) {
        tx[i] = 0U;
    }

    Int_SpiCsAssert();
    if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 1U), 100U) != HAL_OK) {
        Int_SpiCsRelease();
        return 0U;
    }
    Int_SpiCsRelease();

    for (i = 0U; i < len; i++) {
        out[i] = rx[i + 1U];
    }
    return 1U;
}

/* 突发写：首字节发 reg&0x7F，随后依次写数据。 */
uint8_t Int_SpiWriteBuf(uint8_t reg, const uint8_t *in, uint16_t len)
{
    uint8_t  tx[24];
    uint16_t i;

    if ((in == NULL) || (len == 0U) || (len > 23U)) {
        return 0U;
    }

    tx[0] = (uint8_t)(reg & 0x7FU);
    for (i = 0U; i < len; i++) {
        tx[i + 1U] = in[i];
    }

    Int_SpiCsAssert();
    if (HAL_SPI_Transmit(&hspi2, tx, (uint16_t)(len + 1U), 100U) != HAL_OK) {
        Int_SpiCsRelease();
        return 0U;
    }
    Int_SpiCsRelease();
    return 1U;
}
