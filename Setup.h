/**
 * @file Setup.h
 * @brief 选择编译操作系统的配置部分
 * @author RedlightASl (dddbbbdd@foxmail.com)
 * @version 1.0
 * @date 2021-09-30
 *
 * @copyright Copyright (c) 2021  RedlightASl
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-09-30 <td>1.0     <td>wangh     <td>Content
 * </table>
 */
#ifndef __ROV_SETUP_H
#define __ROV_SETUP_H

/* 硬件设置 */
/* 是否使用FPU */
#if /* ARMCC */ (  (defined ( __CC_ARM ) && defined ( __TARGET_FPU_VFP ))    \
    /* Clang */ || (defined ( __CLANG_ARM ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)) \
    /* IAR */   || (defined ( __ICCARM__ ) && defined ( __ARMVFP__ ))        \
    /* GNU */   || (defined ( __GNUC__ ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)) )
#define USING_FPU
#endif

/* 推进器数量 */
#define NUMBER_OF_VERTICAL_THRUSTER 2
#define NUMBER_OF_HORIZENTAL_THRUSTER 4

/* 选择运行仓位 */
#define CONTROL_CARBIN //控制仓

//#define MOVE_CARBIN //PWM仓、电源仓

/* 硬件加速开关 */
#define HARDWARE_ACCELERATE_PID
#define HARDWARE_ACCELERATE_KALMAN
#define HARDWARE_ACCELERATE_CRC
#define HARDWARE_ACCELERATE_PARITY
#define HARDWARE_ACCELERATE_XOR
#define HARDWARE_ACCELERATE_SUM
#define HARDWARE_ACCELERATE_SENSOR

/* 内核设置 */

/* 内核名称最大长度 */
#define NITORI_CORE_NAME_MAX_LENGTH         8
/* 默认线程栈大小 */
#define NITORI_STACK_SIZE_DEFAULT	    	512
/* 默认线程优先级 */
#define NITORI_THREAD_PRIORITY_DEFAULT       3
/* 最大线程优先级 */
#define	NITORI_MAX_THREAD_PRIORITY          8
/* 时钟节拍的周期，以ms为单位，一般为10ms */
#define NITORI_SYSTICK_MS                   10
/* 每个线程最大运行的时间片计数 */
#define NITORI_MAX_RUNTIME_SLICE            10
/* 空闲线程栈大小 */
#define NITORI_IDLETASK_STACK_SIZE	    	512


/* 使用线程调度器 */
#define NITORI_THREAD
/* 使用内核对象(包括信号量互斥量和消息队列) */
#ifdef NITORI_THREAD
#define NITORI_CORE_OBJECT
#endif
/* 使用内核设备 */
#ifdef NITORI_CORE_OBJECT
#define NITORI_DEVICE
#endif

/* 基于HAL库 */
#define NITORI_ON_STM32_HAL

/* 软件设置 */
/* PID模式 */
// #define PID_FIXED //恒定参数

/* 数据校验开关 */
#define DATA_CHECK //数据校验总控制
#define ROV_MESSAGE_DATA_CHECK //上传数据校验
#define ROV_COMMAND_DATA_CHECK //下传指令校验

/* 传感器数据最大长度 */
#define MAX_LENGTH_OF_SENSOR_DATA 16






#endif
