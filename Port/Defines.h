/**
 * @file Defines.h
 * @brief 进行各种基础数据类型的定义
 * @author RedlightASl (dddbbbdd@foxmail.com)
 * @version 1.0
 * @date 2021-09-13
 *
 * @copyright Copyright (c) 2021  RedlightASl
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-09-13 <td>1.0     <td>wangh     <td>Content
 * </table>
 */
#ifndef __ROV_DEFINES_H
#define __ROV_DEFINES_H

 /* 操作系统版本 */
#define NITORI_VERSION 0L
#define NITORI_SUBVERSION 1L
#define NITORI_REVISION 1L
#define NITORI_VERSION ((NITORI_VERSION * 10000) + (NITORI_SUBVERSION * 100) + (NITORI_REVISION))

/* 编程数据类型 */
typedef unsigned            char        u8;                     /**<  8bit integer type */
typedef unsigned            short       u16;                    /**< 16bit integer type */
typedef unsigned            int         u32;                    /**< 32bit integer type */
typedef signed              char        s8;                     /**<  8bit signed integer type */
typedef signed              short       s16;                    /**< 16bit signed integer type */
typedef signed              int         s32;                    /**< 32bit signed integer type */
typedef volatile unsigned   char        vu8;                    /**<  8bit IO__ integer type */
typedef volatile unsigned   short       vu16;                   /**< 16bit IO__ integer type */
typedef volatile unsigned   int         vu32;                   /**< 32bit IO__ integer type */
typedef                     float       f32;                    /**< 32bit single float type */
typedef                     double      f64;                    /**< 64bit double float type */

/* 系统数据类型 */
typedef                     long        rov_BaseType;           /**< 64bit basic type for Nitori Core */
typedef unsigned            long        rov_uBaseType;          /**< 64bit basic type for Nitori Core */
typedef u32                             rov_TaskStackType;      /**< 64bit basic type for Nitori Core */

#define U8_MAX                          0xff                    /**< Maxium number of u8 */
#define U16_MAX                         0xffff                  /**< Maxium number of u16 */
#define U32_MAX                         0xffffffff              /**< Maxium number of u32 */

#define TICK_MAX                        U32_MAX                 /**< Maxium number of sys-tick */

#define ROV_TRUE                        1U                      /**< True Value */
#define ROV_FALSE                       0U                      /**< False Value */
#define ROV_NULL                        (0)                     /**< Null Value */

/* 寄存器数据类型 */
#define MEM32(addr)                     *(volatile unsigned long *)(addr)  /**< 32bit register */
#define MEM8(addr)                      *(volatile unsigned char *)(addr)  /**< 8bit register */

/* 系统控制类型 */
#define ROV_STACK_ALIGN_SIZE            sizeof(u32)
#define ROV_ALIGN(size, align)          (((size) + (align) - 1) & ~((align) - 1))
#define ROV_ALIGN_DOWN(size, align)     ((size)                 & ~((align) - 1))

#if defined (__GNUC__) /* gcc */
#define ROV_STABLE_MEMORY_SPACE         __attribute__((section(".RAM_D1")))
#define SECTION(x)                      __attribute__((section(x)))
#define ROV_UNUSED                      __attribute__((unused))
#define ROV_USED                        __attribute__((used))
#define ALIGN(x)                        __attribute__((aligned(x)))
#define ROV_WEAK                        __attribute__((weak))
#define ROV_INLINE                      __inline
#define ROV_ALWAYS_INLINE               __attribute__((always_inline))
#define ROV_NO_OPTIMIZE                 __attribute__((packed))
#elif defined(__CC_ARM) || defined(__CLANG_ARM) /* clang */
#define ROV_STABLE_MEMORY_SPACE         __attribute__((section(".RAM_D1")))
#define SECTION(x)                      __attribute__((section(x)))
#define ROV_UNUSED                      __attribute__((unused))
#define ROV_USED                        __attribute__((used))
#define ALIGN(x)                        __attribute__((aligned(x)))
#define ROV_WEAK                        __attribute__((weak))
#define ROV_INLINE                      __inline
#define ROV_ALWAYS_INLINE               __attribute__((always_inline))
#define ROV_NO_OPTIMIZE                 __attribute__((packed))
#endif

/* 断言 */
#define ROV_ASSERT(x)                   if((0) == (x)) {DISABLE_INTERRUPTS();while(1);}

/* 硬件设备数据类型 */
#ifdef NITORI_ON_STM32_HAL
#define UART_Device UART_HandleTypeDef
#define TIMER_Device TIM_HandleTypeDef
#define SPI_Device SPI_HandleTypeDef
#define IIC_Device IIC_HandleTypeDef
#else
//串口设备
#define UART_Device vu32*
//定时器设备
#define TIMER_Device vu32*
//SPI设备
#define SPI_Device vu32*
//IIC设备
#define IIC_Device vu32*
//未知设备
#define Unknown_Device vu32*
#endif

typedef enum
{
    rov_Core_Type_NULL = 0x00,
    rov_Core_Type_Thread = 0x01,
    rov_Core_Type_Semaphore = 0x02,
    rov_Core_Type_Mutex = 0x03,
    rov_Core_Type_Queue = 0x04,
    rov_Core_Type_Device = 0x05,
    rov_Core_Type_Unknown = 0x0c,
    rov_Core_Type_Static = 0x80
} rov_Core_Type; /* 内核对象类型 */


typedef enum
{
    THREAD_STATUS_INIT,
    THREAD_STATUS_READY,
    THREAD_STATUS_RUNNING,
    THREAD_STATUS_SUSPEND,
    THREAD_STATUS_SHUTDOWN,
    THREAD_STATUS_CLOSE
} rov_Thread_Status; /* 线程状态 */

typedef enum
{
    THREAD_CTRL_STARTUP,
    THREAD_CTRL_SHUTDOWN,
    THREAD_CTRL_CHANGE_PRIORITY,
    THREAD_CTRL_GET_INFO
} rov_Thread_Ctrl; /* 线程控制 */

typedef enum
{
    SEMAPHOR_STATUS_INIT,
    SEMAPHOR_STATUS_HANG,
    SEMAPHOR_STATUS_SUSPEND,
    SEMAPHOR_STATUS_RECEIVED
} rov_Semaphore_Status; /* 信号量状态 */

typedef enum
{
    MUTEX_STATUS_INIT,
    MUTEX_STATUS_HANG,
    MUTEX_STATUS_SUSPEND,
    MUTEX_STATUS_RECEIVED
} rov_Mutex_Status; /* 互斥量状态 */

typedef enum
{
    QUEUE_STATUS_INIT,
    QUEUE_STATUS_EMPTY,
    QUEUE_STATUS_FULL,
    QUEUE_STATUS_
} rov_Queue_Status; /* 队列状态 */

typedef enum
{
    DEVICE_CLOSE,
    DEVICE_OPEN,
    DEVICE_ERROR,
    DEVICE_BUSY
} rov_Devce_Status; /* 硬件设备状态 */


typedef enum
{
    ROTATE_MODE,
    SIDEPUSH_MODE,
    MIX_MODE
} HorizentalMode; /* 水平推进器模式 */


typedef enum
{
    UPDOWN_MODE,
    ROLL_MODE,
    PITCH_MODE,
    MIX_MODE
} VerticalMode; /* 垂直推进器模式 */

typedef enum
{
    PID_IP, //增量整型
    PID_PP, //位置整型
    PID_IF, //增量浮点
    PID_PF //位置浮点
} rov_Algorithm_Pid; /* PID模式 */



#endif
