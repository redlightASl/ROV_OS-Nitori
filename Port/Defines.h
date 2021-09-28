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

#ifdef __cplusplus
extern "C" {
#endif

/* Distribution Version */
#define NITORI_VERSION 0L
#define NITORI_SUBVERSION 1L
#define NITORI_REVISION 1L

#define NITORI_VERSION ((NITORI_VERSION * 10000) + (NITORI_SUBVERSION * 100) + (NITORI_REVISION))

//串口硬件设备
// #define UART_Device UART_HandleTypeDef
#define UART_Device u8*
//定时器硬件设备
#define TIMER_Device TIM_HandleTypeDef
//SPI硬件设备
#define SPI_Device SPI_HandleTypeDef
//IIC硬件设备
#define IIC_Device IIC_HandleTypeDef

#define ROV_STABLE_MEMORY_SPACE __attribute__((section(".RAM_D1")))
#define ROV_ALIGN(n) __attribute__((aligned(n)))





typedef unsigned            char   u8;                /**<  8bit integer type */
typedef unsigned            short  u16;               /**< 16bit integer type */
typedef unsigned            int    u32;               /**< 32bit integer type */
typedef volatile unsigned   char   vu8;      /**<  8bit IO__ integer type */
typedef volatile unsigned   short  vu16;     /**< 16bit IO__ integer type */
typedef volatile unsigned   int    vu32;     /**< 32bit IO__ integer type */

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

typedef enum
{
    THREAD_STATUS_RUN,
    THREAD_STATUS_SHUTDOWN
} rov_Thread_Status;





enum HorizentalMode{
    ROTATE_MODE,
    SIDEPUSH_MODE,
    MIX_MODE
};

enum VerticalMode{
    UPDOWN_MODE,
    ROLL_MODE,
    PITCH_MODE,
    MIX_MODE
};



#ifdef __cplusplus
}
#endif

#endif
