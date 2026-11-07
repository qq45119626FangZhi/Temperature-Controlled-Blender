/*
 * FreeRTOS Kernel 配置头文件（针对 STM32F103RCT6 / Cortex-M3 / Keil MDK-ARM）
 *
 * 移植说明（本文件由 CubeMX F1 模板适配而来）：
 *  - RTOS 时钟节拍由 SysTick 提供，固定 1kHz（configTICK_RATE_HZ = 1000）。
 *  - HAL 的时基（HAL_Delay / HAL_IncTick）由 TIM6 单独提供（见 .ioc 的
 *    NVIC.TimeBase=TIM6_IRQn），与 RTOS 节拍互不冲突。
 *  - 调用 FreeRTOS "FromISR" 系列 API 的中断，其抢占优先级数值必须不小于 5
 *    （即 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5）。优先级数值小于 5
 *    （0~4，即更高优先级）的中断禁止调用 FreeRTOS API。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* 仅在编译器下包含 stdint，汇编器不需要 */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
 #include <stdint.h>
 extern uint32_t SystemCoreClock;
#endif

/* 基础调度配置 */
#define configUSE_PREEMPTION                    1   /* 抢占式调度 */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configMAX_PRIORITIES                    ( 7 )   /* 用户任务最大优先级数 */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configCPU_CLOCK_HZ                      ( 72000000 )   /* CPU 主频 72MHz */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 ) /* 系统节拍 1kHz，由 SysTick 提供 */
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 128 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 15 * 1024 ) )  /* FreeRTOS 堆 15KB */
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_TRACE_FACILITY                1
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1
#define configGENERATE_RUN_TIME_STATS           0

/* 协程配置（默认关闭） */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/* 软件定时器配置（默认关闭） */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* 可选 API 包含开关 */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 0
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1

/* Cortex-M 优先级位宽（STM32F1 为 4 位，共 16 级） */
#ifdef __NVIC_PRIO_BITS
 #define configPRIO_BITS                        __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS                        4
#endif

/* 最低中断优先级（数值最大），供内核自身使用 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 0xf

/* 可安全调用 FreeRTOS "FromISR" API 的最高（数值最小）中断优先级。
 * 注意：只有抢占优先级数值 >= 5 的中断才能调用 FromISR 系列 API；
 *       优先级数值 < 5（更高优先级）的中断禁止调用 FreeRTOS API。 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* 内核自身中断优先级（SysTick / PendSV 使用最低优先级） */
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY 不能设为 0 !!!!
   详见 http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* 断言：出错时关闭中断并死循环（可按需替换为自己的错误处理） */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* 将 FreeRTOS 移植层的中断处理函数映射到 CMSIS 标准名称，
 * 以覆盖启动文件中同名的弱定义（SysTick / PendSV / SVC）。 */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
