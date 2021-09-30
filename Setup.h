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

/* 是否使用FPU */
#define USING_FPU

/* 选择运行仓位 */
#define CONTROL_CARBIN //控制仓
//#define MOVE_CARBIN //PWM仓、电源仓

/* 数据校验开关 */
#define DATA_CHECK //数据校验总控制
#define ROV_MESSAGE_DATA_CHECK //上传数据校验
#define ROV_COMMAND_DATA_CHECK //下传指令校验

/* 硬件加速开关 */
#define HARDWARE_ACCELERATE_PID
#define HARDWARE_ACCELERATE_KALMAN
#define HARDWARE_ACCELERATE_CRC
#define HARDWARE_ACCELERATE_PARITY
#define HARDWARE_ACCELERATE_XOR
#define HARDWARE_ACCELERATE_SUM
#define HARDWARE_ACCELERATE_SENSOR

/* PID模式 */
// #define PID_FIXED //恒定参数
#ifdef PID_FIXED
/* 恒定PID参数 */
//定深比例系数
#define PID_DEEP_Kp 200
//定深积分系数
#define PID_DEEP_Ki 2
//定深微分系数
#define PID_DEEP_Kd 300
//定向比例系数
#define PID_ORBIT_Kp 200
//定向积分系数
#define PID_ORBIT_Ki 2
//定向微分系数
#define PID_ORBIT_Kd 300
#else
#define PID_ADJUSTABLE //使用上位机下传的参数
#endif

/* 推进器数量 */
#define NUMBER_OF_VERTICAL_THRUSTER 2
#define NUMBER_OF_HORIZENTAL_THRUSTER 4

/* 传感器数据最大长度 */
#define MAX_LENGTH_OF_SENSOR_DATA 8

/* 内核名称最大长度 */
#define MAX_LENGTH_OF_CORE_NAME 8




#endif
