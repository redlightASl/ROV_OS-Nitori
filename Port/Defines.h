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

#define ROV_STABLE_MEMORY_SPACE __attribute__((section(".RAM_D1")))

typedef unsigned   char   u8;                /**<  8bit integer type */
typedef unsigned   short  u16;               /**< 16bit integer type */
typedef unsigned   int    u32;               /**< 32bit integer type */
typedef volatile unsigned   char   vu8;      /**<  8bit IO__ integer type */
typedef volatile unsigned   short  vu16;     /**< 16bit IO__ integer type */
typedef volatile unsigned   int    vu32;     /**< 32bit IO__ integer type */

#endif
