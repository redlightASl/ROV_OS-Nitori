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
#include <Setup.h>

 /* 操作系统版本 */
#define NITORI_VERSION 0L
#define NITORI_SUBVERSION 1L
#define NITORI_REVISION 1L
#define NITORI_VERSION ((NITORI_VERSION * 10000) + (NITORI_SUBVERSION * 100) + (NITORI_REVISION))

/* 硬件设备 */
//串口设备
// #define UART_Device UART_HandleTypeDef
#define UART_Device u8*
//定时器设备
// #define TIMER_Device TIM_HandleTypeDef
#define TIMER_Device u8*
//SPI设备
// #define SPI_Device SPI_HandleTypeDef
#define SPI_Device u8*
//IIC设备
// #define IIC_Device IIC_HandleTypeDef
#define IIC_Device u8*
//未知设备
#define Unknown_Device u8*

/* 数据类型 */
typedef unsigned            char        u8;             /**<  8bit integer type */
typedef unsigned            short       u16;            /**< 16bit integer type */
typedef unsigned            int         u32;            /**< 32bit integer type */
typedef volatile unsigned   char        vu8;            /**<  8bit IO__ integer type */
typedef volatile unsigned   short       vu16;           /**< 16bit IO__ integer type */
typedef volatile unsigned   int         vu32;           /**< 32bit IO__ integer type */
typedef float                           f32;            /**< 32bit single float type */
typedef double                          f64;            /**< 64bit double float type */

#define U8_MAX                          0xff            /**< Maxium number of u8 */
#define U16_MAX                         0xffff          /**< Maxium number of u16 */
#define U32_MAX                         0xffffffff      /**< Maxium number of u32 */
#define TICK_MAX                        U32_MAX         /**< Maxium number of sys-tick */

#define ROV_STABLE_MEMORY_SPACE         __attribute__((section(".RAM_D1")))
#define SECTION(x)                      __attribute__((section(x)))
#define ROV_UNUSED                      __attribute__((unused))
#define ROV_USED                        __attribute__((used))
#define ALIGN(x)                        __attribute__((aligned(x)))
#define ROV_WEAK                        __attribute__((weak))
#define ROV_ALIGN(size, align)          (((size) + (align) - 1) & ~((align) - 1))
#define ROV_ALIGN_DOWN(size, align)     ((size)                 & ~((align) - 1))

/* 内核对象类型 */
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
} rov_Core_Type;

/* 线程状态 */
typedef enum
{
    THREAD_STATUS_RUN,
    THREAD_STATUS_SHUTDOWN
} rov_Thread_Status;


/* 硬件设备状态 */




/* 水平推进器模式 */
enum HorizentalMode {
    ROTATE_MODE,
    SIDEPUSH_MODE,
    MIX_MODE
};

/* 垂直推进器模式 */
enum VerticalMode {
    UPDOWN_MODE,
    ROLL_MODE,
    PITCH_MODE,
    MIX_MODE
};





#endif
