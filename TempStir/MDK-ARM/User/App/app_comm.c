/*
 * @file    app_comm.c
 * @brief   P6：通信通道管理、写命令仲裁、统一写入口、UART 接收分发。
 *
 * 通道与资源（引脚以 SCH_主控_2_2026-08-27new.pdf 为准）：
 *   RS485/Modbus RTU -> UART4(PC10=MCU_485_TX / PC11=MCU_485_RX) + MAX13487E（自动收发方向）
 *   蓝牙透传         -> USART2(PA2=BL-RXD / PA3=BL-TXD) + ECB01C
 *   CAN              -> 本期未实现（D4）
 *
 * 数据交换：所有对外量都映射到 app_modbus.h 的保持寄存器表；
 * 读走各通道自己的取数逻辑（蓝牙复用 App_ModbusReadReg），写一律走
 * App_CommWriteReg()，由本文件完成 —— 面板仲裁(8s) -> 范围校验 -> 保护联锁 -> 下发。
 */
#include "app_comm.h"
#include "app_main.h"
#include "app_store.h"
#include "app_modbus.h"
#include "app_ble.h"
#include "dri_ble.h"     /* USART2 单字节接收/错误回调（模块驱动层） */
#include "app_heat.h"
#include "app_motor.h"   /* P7：清堵转锁存 */
#include "app_temp.h"
#include "app_fault.h"
#include "app_rtc.h"
#include "app_imu.h"     /* P9：IMU 参数/零偏标定/事件清除 */
#include "app_tune.h"    /* A2：实测统计控制 */
#include "menu_ui.h"
#include "int_gpio.h"
#include "app_freemodbus_port.h"
#include "usart.h"

#include <stdio.h>

/* RS485 可用波特率表：代码 0~4 对应 9600/19200/38400/57600/115200 */
static const uint32_t s_baudTable[5] =
{
    9600U, 19200U, 38400U, 57600U, 115200U
};

static uint8_t  s_channels = APP_COMM_CH_DEFAULT;
static uint16_t s_slaveAddr = 1U;
static uint32_t s_baud = 9600U;
static uint16_t s_baudCode = 0U;

/* 面板占用窗口 */
static uint32_t s_panelTick = 0U;
static uint8_t  s_panelSeen = 0U;

/* ---------------- 面板活动与仲裁 ---------------- */

void App_CommNotifyPanelKey(void)
{
    s_panelTick = (uint32_t)xTaskGetTickCount();
    s_panelSeen = 1U;
}

uint8_t App_CommIsPanelBusy(void)
{
    if (s_panelSeen == 0U)
    {
        return 0U;
    }

    if ((uint32_t)(xTaskGetTickCount() - s_panelTick) < pdMS_TO_TICKS(APP_COMM_PANEL_HOLD_MS))
    {
        return 1U;
    }

    return 0U;
}

/* ---------------- 配置查询 ---------------- */

uint8_t App_CommGetChannels(void)
{
    return s_channels;
}

uint8_t App_CommGetSlaveAddr(void)
{
    return (uint8_t)s_slaveAddr;
}

uint32_t App_CommGetBaud(void)
{
    return s_baud;
}

uint16_t App_CommGetBaudCode(void)
{
    return s_baudCode;
}

/* ---------------- 故障码 ---------------- */

uint16_t App_CommGetFaultBits(void)
{
    /* P7：故障字统一由 app_fault 聚合（监控任务与通信共用同一份判定），
     * 这里只做转发，避免两处逻辑漂移。 */
    return App_FaultGetBits();
}

/* ---------------- 控制命令字 ---------------- */

/* 电机启动联锁只关心的故障位：P7 电机堵转（bit5，尚未启用）。
 * 加热类(bit0~bit4)、EEPROM(bit6)、RTC 丢时(bit7) 不拦电机启动。 */
#define APP_COMM_FAULT_MOTOR_BLOCK   APP_MB_FLT_MOTOR_STALL

static App_CommResult_t App_CommExecCommand(uint16_t cmd)
{
    if ((cmd & (uint16_t)(~APP_MB_CMD_VALID_MASK)) != 0U)
    {
        return APP_COMM_ERR_VALUE;   /* 未定义的命令位 */
    }

    if ((cmd & APP_MB_CMD_START) != 0U)
    {
        /* 保护联锁：只用与电机安全相关的故障位拦启动（P7 堵转 bit5）。
         * 加热类故障（bit0~bit4，含铝板限温 0x0010）、EEPROM(bit6)、RTC 丢时(bit7)
         * 与电机无关：加热已由 App_HeatControlStep() 自行关断，
         * 不能再连带禁止电机启动（否则会误报 04 且只能靠上位机猜原因）。 */
        if ((App_CommGetFaultBits() & APP_COMM_FAULT_MOTOR_BLOCK) != 0U)
        {
            return APP_COMM_ERR_VALUE;
        }
        Menu_UiCommSetRunning(1U);
    }

    if ((cmd & APP_MB_CMD_STOP) != 0U)
    {
        Menu_UiCommSetRunning(0U);
    }

    if ((cmd & APP_MB_CMD_CLEAR_FAULT) != 0U)
    {
        Int_GpioFaultReset();            /* 释放加热硬件故障锁存 */
        App_HeatClearPlateLimit();       /* 同时清铝板限温锁存（否则只能等降温/重启） */
        App_MotorClearStall();           /* P7：清堵转锁存（清完才能重新启动电机） */
        (void)App_StoreFlushSave();
    }

    if ((cmd & APP_MB_CMD_SAVE_PARAM) != 0U)
    {
        App_StoreRequestSave();
        (void)App_StoreFlushSave();      /* 不等 2s 静置，立即落盘 */
    }

    if ((cmd & APP_MB_CMD_LOAD_DEFAULT) != 0U)
    {
        App_Store_Params_t def;

        App_StoreDefault(&def);
        App_StoreSetUi(def.speed, def.plateTemp, def.liquidTemp, def.runTimeSec);
        App_StoreSetMotorPid(def.motorPidKp, def.motorPidKi, def.motorPidKd);
        App_StoreSetHeatPid(def.heatPidKp, def.heatPidKi, def.heatPidKd);
        App_StoreSetCalib(def.encoderPpr, def.gearRatioX100, def.liquidOffsetC10, def.boardOffsetC10);
        App_StoreRequestSave();
        (void)App_StoreFlushSave();
        Menu_UiCommReloadParams();       /* 面板同步显示默认值 */
    }

    return APP_COMM_OK;
}

/* ---------------- 统一写入口 ---------------- */

App_CommResult_t App_CommWriteReg(uint16_t addr, uint16_t value)
{
    /* 1) 面板仲裁：面板刚被操作过，通信写入先让路。
     *    焦点项只改高亮位置，不动执行器，故不受 8s 窗口限制（否则 App 的 KEY1 会回 06）。 */
    if ((App_CommIsPanelBusy() != 0U) && (addr != APP_MB_REG_UI_FOCUS))
    {
        return APP_COMM_ERR_BUSY;
    }

    switch (addr)
    {
        case APP_MB_REG_UI_FOCUS:
            if (value > 3U)
            {
                return APP_COMM_ERR_VALUE;
            }
            Menu_UiCommSetFocus((uint8_t)value);   /* 等价面板 KEY1：OLED 箭头随动 */
            return APP_COMM_OK;

        case APP_MB_REG_PLATE_LIMIT:
            if (value > (uint16_t)MENU_UI_PLATE_MAX)
            {
                return APP_COMM_ERR_VALUE;
            }
            if ((value > 0U) && ((int32_t)value < (APP_HEAT_PLATE_LIMIT_MIN_C10 / 10)))
            {
                /* 低于 35℃ 的铝板上限在室温下就会锁存且永远无法回差解除，
                 * 会把加热永久关断，直接拒绝（0 = 不限制 或 >= 35℃） */
                printf("[P6] plate limit %u rejected (use 0 or >=%d)\r\n",
                       (unsigned)value, (int)(APP_HEAT_PLATE_LIMIT_MIN_C10 / 10));
                return APP_COMM_ERR_VALUE;
            }
            Menu_UiCommSetPlateLimit((int32_t)value);  /* 与面板铝板行等价，含限温锁存复位 */
            return APP_COMM_OK;
        case APP_MB_REG_TARGET_TEMP:
            if (value > (uint16_t)(MENU_UI_LIQUID_MAX * 10))
            {
                return APP_COMM_ERR_VALUE;
            }
            Menu_UiCommSetLiquid((int32_t)(value / 10U));
            return APP_COMM_OK;

        case APP_MB_REG_TARGET_SPEED:
            if (value > (uint16_t)MENU_UI_SPEED_MAX)
            {
                return APP_COMM_ERR_VALUE;
            }
            Menu_UiCommSetSpeed((int32_t)value);
            return APP_COMM_OK;

        case APP_MB_REG_SET_TIME:
            if (value > (uint16_t)MENU_UI_TIME_MAX)
            {
                return APP_COMM_ERR_VALUE;
            }
            Menu_UiCommSetTime((uint32_t)value);
            return APP_COMM_OK;

        case APP_MB_REG_CTRL_CMD:
            return App_CommExecCommand(value);

        case APP_MB_REG_SLAVE_ADDR:
            if ((value < 1U) || (value > 247U))
            {
                return APP_COMM_ERR_VALUE;
            }
            s_slaveAddr = value;
            App_StoreSetComm(s_baud, s_slaveAddr);
            App_StoreRequestSave();
            printf("[P6] slave addr -> %u (reboot to take effect)\r\n", (unsigned)value);
            return APP_COMM_OK;

        case APP_MB_REG_BAUD_CODE:
            if (value > 4U)
            {
                return APP_COMM_ERR_VALUE;
            }
            s_baudCode = value;
            s_baud = s_baudTable[value];
            App_StoreSetComm(s_baud, s_slaveAddr);
            App_StoreRequestSave();
            printf("[P6] baud -> %lu (reboot to take effect)\r\n", (unsigned long)s_baud);
            return APP_COMM_OK;

        /* ---------------- P9：六轴 / 斜坡参数 ---------------- */
        case APP_MB_REG_IMU_CTRL:
            /* bit1 零偏标定 / bit2 保存参数 / bit3 清事件；其余位忽略 */
            if ((value & APP_MB_IMUCTL_CAL_BIAS) != 0U)
            {
                App_ImuStartBiasCal();
            }
            if ((value & APP_MB_IMUCTL_SAVE) != 0U)
            {
                (void)App_ImuSaveParams();
            }
            if ((value & APP_MB_IMUCTL_CLEAR_EVT) != 0U)
            {
                App_ImuClearEvents();
            }
            return APP_COMM_OK;

        case APP_MB_REG_ACC_CONFIG:
        case APP_MB_REG_GYRO_CONFIG:
        {
            /* 低字节量程编码(0~3)、高字节 ODR 编码(0~9)；改后重启生效
             * （量程/ODR 只能在 Lsm6dsm_Init 里整组下发，运行期不改采样链）。 */
            uint8_t fsCode = (uint8_t)(value & 0x00FFU);
            uint8_t odrCode = (uint8_t)((value >> 8) & 0x00FFU);

            if ((fsCode > 3U) || (odrCode > 9U))
            {
                return APP_COMM_ERR_VALUE;
            }
            if (addr == APP_MB_REG_ACC_CONFIG)
            {
                ((App_Imu_Params_t *)App_ImuGetParams())->accFs = fsCode;
            }
            else
            {
                ((App_Imu_Params_t *)App_ImuGetParams())->gyroFs = fsCode;
            }
            ((App_Imu_Params_t *)App_ImuGetParams())->odrCode = odrCode;
            (void)App_ImuSaveParams();
            printf("[P9] imu cfg fs=%u odr=%u saved (reboot to take effect)\r\n",
                   (unsigned)fsCode, (unsigned)odrCode);
            return APP_COMM_OK;
        }

        case APP_MB_REG_ALPHA_LIMIT:
            App_ImuSetAlphaLimitRpmS(value);
            (void)App_ImuSaveParams();
            return APP_COMM_OK;

        case APP_MB_REG_SLIP_TH:
            App_ImuSetSlipThX10(value);
            (void)App_ImuSaveParams();
            return APP_COMM_OK;

        case APP_MB_REG_VIB_TH:
            App_ImuSetVibThMg(value);
            (void)App_ImuSaveParams();
            return APP_COMM_OK;

        case APP_MB_REG_TILT_TH:
            App_ImuSetTiltThC10(value);
            (void)App_ImuSaveParams();
            return APP_COMM_OK;

        /* ---------------- v1.3（A2/A4）：现场可调参数 ---------------- */
        case APP_MB_REG_HEAT_OC:
            /* 加热过流阈值：ADC raw 100~4095（与面板标定页 / EEPROM 同一份数据） */
            if ((value < 100U) || (value > 4095U))
            {
                return APP_COMM_ERR_VALUE;
            }
            App_HeatSetOverCurrentThreshold(value);
            App_StoreSetHeatOc(value);
            App_StoreRequestSave();
            printf("[P11] heat OC -> %u (saved)\r\n", (unsigned)value);
            return APP_COMM_OK;

        case APP_MB_REG_MOTOR_ASSIST:
            /* 堵转辅助：bit0 = 启用振动辅助判定；其余位未定义，直接拒绝避免误写 */
            if ((value & (uint16_t)(~APP_MB_ASSIST_STALL_EN)) != 0U)
            {
                return APP_COMM_ERR_VALUE;
            }
            App_MotorSetStallAssistEnable(((value & APP_MB_ASSIST_STALL_EN) != 0U) ? 1U : 0U);
            App_ImuSetSlipAssistStall(((value & APP_MB_ASSIST_STALL_EN) != 0U) ? 1U : 0U);
            (void)App_ImuSaveParams();
            return APP_COMM_OK;

        case APP_MB_REG_TUNE_CTRL:
            /* 实测统计：bit0 周期打印 / bit1 立即打印 / bit2 清统计（后两位自清零） */
            if ((value & (uint16_t)(~(APP_MB_TUNECTL_PRINT | APP_MB_TUNECTL_ONCE | APP_MB_TUNECTL_CLEAR))) != 0U)
            {
                return APP_COMM_ERR_VALUE;
            }
            (void)App_TuneSetCtrl(value);
            return APP_COMM_OK;

        default:
            /* 只读寄存器或未定义地址 */
            return APP_COMM_ERR_ADDR;
    }
}

/* ---------------- UART 接收分发 ---------------- */

/* HAL 弱函数实现：所有串口接收完成都从这里分发。
 * UART4(PC10/PC11) -> FreeModbus RTU(RS485)；USART2(PA2/PA3) -> 蓝牙 ECB01C。
 * USART1 只用于 printf 发送，不参与。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        App_FreeModbusSerialRxCpltCallback(huart);
    }
    else if (huart->Instance == USART2)
    {
        Ble_RxCpltCallback(huart);
    }
    else
    {
        /* 其它串口不再挂接收 */
    }
}

/* 接收错误（溢出/噪声等）：重新挂接收，避免通道永久失聪。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        App_FreeModbusSerialRxErrorCallback();
    }
    else if (huart->Instance == USART2)
    {
        Ble_RxErrorCallback(huart);
    }
    else
    {
        /* 其它串口忽略 */
    }
}

/* ---------------- 初始化与任务 ---------------- */

void App_CommInit(void)
{
    const App_Store_Params_t *p = App_StoreGet();

    s_channels = p->commEn;
    if ((s_channels == 0U) || (s_channels == 0xFFU))
    {
        s_channels = APP_COMM_CH_DEFAULT;   /* 旧数据/异常值 -> 默认开 RS485 + 蓝牙 */
    }

    s_slaveAddr = p->slaveAddr;
    if ((s_slaveAddr < 1U) || (s_slaveAddr > 247U))
    {
        s_slaveAddr = 1U;
    }

    s_baud = p->modbusBaud;
    s_baudCode = 0U;
    if (s_baud == 0U)
    {
        s_baud = 9600U;
    }
    for (uint8_t i = 0U; i < 5U; i++)
    {
        if (s_baudTable[i] == s_baud)
        {
            s_baudCode = i;
            break;
        }
    }

    printf("[P6] comm ch=0x%02X addr=%u baud=%lu\r\n",
           (unsigned)s_channels, (unsigned)s_slaveAddr, (unsigned long)s_baud);

    if ((s_channels & APP_COMM_CH_RS485) != 0U)
    {
        /* 按 EEPROM 里的波特率重配 UART4（CubeMX 默认 115200，RS485 要 9600） */
        huart4.Init.BaudRate = s_baud;
        if (HAL_UART_Init(&huart4) != HAL_OK)
        {
            printf("[P6] UART4 re-init FAIL\r\n");
        }

        if (App_ModbusStart(s_slaveAddr, s_baud) != 0U)
        {
            printf("[P6] Modbus RTU slave ready (UART4/RS485 PC10-PC11)\r\n");
        }
        else
        {
            printf("[P6] Modbus RTU start FAIL\r\n");
        }
    }

    if ((s_channels & APP_COMM_CH_BT) != 0U)
    {
        App_BleInit();
        printf("[P6] BLE passthrough ready (USART2 PA2-PA3, Modbus RTU frames)\r\n");
    }
}

void App_CommTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        App_ModbusPoll();   /* 内部阻塞等待协议事件（接收完成/定时器超时） */
    }
}

void App_BleTask(void *argument)
{
    uint8_t ready = 0U;

    (void)argument;

    for (;;)
    {
        App_BlePoll();

        if (ready == 0U)
        {
            /* 握手 + 改名（幂等，已等于目标名不会重复写）。
             * 未成功则每 5s 重试一次：模块上电慢、热复位晚或中途掉电都能自动恢复，
             * 不需要重启整机。 */
            if (App_BleBringUp() != 0U)
            {
                ready = 1U;
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(5000U));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}
