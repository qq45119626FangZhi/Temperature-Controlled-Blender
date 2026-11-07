/*
 * @file    app_main.h
 * @brief   业务应用层（MDK-ARM/User/App）对 FreeRTOS 的封装与验证入口。
 *
 * 设计要点（对应本次移植要求）：
 *  - 所有 FreeRTOS 对象（任务、通知句柄）的创建都集中在 App_Main_Init()，
 *    必须在 vTaskStartScheduler() 之前调用（规则 12）。
 *  - 中断中只允许“置位 / 队列投递 / 任务通知”，不允许 printf、阻塞式 HAL、
 *    协议解析或复杂计算（规则 6）。
 *  - 调用 FreeRTOS FromISR 系列 API 的中断，其抢占优先级数值必须不小于 5
 *    （configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5）。本文件用编译期
 *    断言保证这一点（需求 3）。
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "FreeRTOS.h"
#include "task.h"

/* DS_INT 复用到 EXTI15_10，.ioc 中其抢占优先级为 6。
 * 该中断会调用 FromISR 系列 API，因此优先级数值必须 >= 5。
 * 若低于 5，编译期直接报错，提示违反了优先级限制。 */
#define APP_ISR_PREEMPT_PRIORITY          6

#if ( APP_ISR_PREEMPT_PRIORITY < configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY )
#error "调用 FreeRTOS FromISR API 的中断，其优先级数值必须不小于 5 (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)。"
#endif

/* 应用任务优先级枚举（1~5，数值越大优先级越高）。
 * FreeRTOS 中优先级 0 通常留给空闲任务；configMAX_PRIORITIES = 7 时
 * 合法范围为 0~6，故 1~5 全部有效。使用枚举可避免散落的魔法数字，
 * 便于统一管理与后续调整。 */
typedef enum
{
  APP_TASK_PRIO_1 = 1,   /* 最低应用优先级 */
  APP_TASK_PRIO_2 = 2,
  APP_TASK_PRIO_3 = 3,
  APP_TASK_PRIO_4 = 4,
  APP_TASK_PRIO_5 = 5    /* 最高应用优先级（本枚举范围内） */
} App_TaskPriority_t;

/* 编译期校验：最高优先级 5 必须严格小于 configMAX_PRIORITIES，
 * 防止日后调低 configMAX_PRIORITIES 导致优先级越界。 */
#if ( APP_TASK_PRIO_5 >= configMAX_PRIORITIES )
#error "APP_TASK_PRIO_5 超出 configMAX_PRIORITIES 允许范围（应 < configMAX_PRIORITIES）。"
#endif

/* 任务栈大小枚举：以 128 字（StackType_t，Cortex-M3 为 4 字节）为基础，
 * 提供 1x / 2x / 4x / 8x / 10x 五档；与 xTaskCreate 的 usStackDepth 单位一致。
 * 默认创建任务使用 2 倍（256 字 = 1KB）。 */
typedef enum
{
  APP_TASK_STACK_1X      = 128,        /* 1x  = 128  字（基础） */
  APP_TASK_STACK_2X      = 128 * 2,    /* 2x  = 256  字（默认） */
  APP_TASK_STACK_4X      = 128 * 4,    /* 4x  = 512  字 */
  APP_TASK_STACK_8X      = 128 * 8,    /* 8x  = 1024 字 */
  APP_TASK_STACK_10X     = 128 * 10,   /* 10x = 1280 字 */
  APP_TASK_STACK_DEFAULT = APP_TASK_STACK_2X   /* 默认创建任务：2 倍 */
} App_TaskStack_t;

/* ---------------- 全局状态（App_State_t） ---------------- */

/* UI 焦点：当前 UI 正在编辑/聚焦的目标项。 */
typedef enum
{
  APP_UI_FOCUS_SPEED  = 0,  /* 焦点：速度 */
  APP_UI_FOCUS_TEMP   = 1,  /* 焦点：液体温度 */
  APP_UI_FOCUS_TIME   = 2,  /* 焦点：时间（预留） */
  APP_UI_FOCUS_PLATE  = 3   /* 焦点：铝板温度 */
} App_UiFocus_t;

/* 系统状态：空闲 / 运行 / 故障。 */
typedef enum
{
  APP_SYS_IDLE    = 0,      /* 空闲 */
  APP_SYS_RUNNING = 1,      /* 运行 */
  APP_SYS_FAULT   = 2       /* 故障 */
} App_SysState_t;

/* 设备全局状态：所有任务共享。
 * 速度、温度均为整数（单位由业务约定：速度 rpm、温度 ℃）。 */
typedef struct
{
  uint32_t         device_id;      /* 设备唯一 ID（由 STM32 96bit UID 计算） */
  int32_t          cur_speed;      /* 当前真实速度（整数） */
  int32_t          target_speed;   /* 目标速度（整数） */
  int32_t          cur_temp;       /* 当前真实温度（整数，℃） */
  int32_t          target_temp;    /* 目标温度（整数，℃） */
  uint32_t         target_time;    /* 目标运行时间（秒） */
  uint32_t         remain_time;    /* 剩余运行时间（秒） */
  App_UiFocus_t    ui_focus;       /* UI 焦点：速度/温度/时间 */
  App_SysState_t   sys_state;      /* 系统状态：空闲/运行/故障 */
} App_State_t;

/* 全局状态实例（命名规则：app_main_ 前缀 + 小写）。 */
extern App_State_t app_main_state;

/* 初始化全局状态：默认值清零 → 由 STM32 UID 计算 device_id → 创建互斥锁。
 * 必须在 vTaskStartScheduler() 之前调用（互斥锁需在调度器启动前创建）。 */
void App_Main_StateInit(void);

/* 全局状态互斥锁接口：加锁 / 释放锁（内部用 portMAX_DELAY 阻塞等待）。
 * 任务读写 app_main_state 前应调用 App_Main_StateLock()，完成后调用
 * App_Main_StateUnlock()，避免多任务抢占导致数据竞争。 */
void App_Main_StateLock(void);
void App_Main_StateUnlock(void);

/* 创建 FreeRTOS 对象（任务 / 通知句柄）。
 * 必须在 vTaskStartScheduler() 之前调用（规则 12）。 */
void App_Main_Init(void);

/* 供中断服务程序调用：向已挂接的任务投递一个任务通知。
 * 仅做任务通知（置位），不执行阻塞或复杂处理（符合规则 6）。 */
void App_Main_NotifyFromISR(void);

/* 挂接 DS_INT（DS3231 1Hz 秒节拍）通知的接收任务：在该任务体内传
 * xTaskGetCurrentTaskHandle() 调用一次即可；传 NULL 解除挂接。
 * 未挂接时 App_Main_NotifyFromISR() 直接返回（不通知）。 */
void App_Main_AttachNotifyTask(TaskHandle_t task);

#endif /* APP_MAIN_H */
