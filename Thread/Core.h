/**
 * @file Core.h
 * @brief 用于构建线程、信号量、消息队列、系统服务的基础内核类
 * @author RedlightASl (dddbbbdd@foxmail.com)
 * @version 1.0
 * @date 2021-10-05
 *
 * @copyright Copyright (c) 2021  RedlightASl
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-10-05 <td>1.0     <td>wangh     <td>Content
 * </table>
 */
#ifndef __ROV_CORE_H
#define __ROV_CORE_H
#include "Defines.h"

//XXX:链表部分已经完成
struct rov_SingleList
{
    struct rov_SingleList* next;
};
typedef struct rov_SingleList rov_SingleList_t; /* 单链表ADT */

struct rov_DoubleList
{
    struct rov_DoubleList* prev;
    struct rov_DoubleList* next;
};
typedef struct rov_DoubleList rov_DoubleList_t; /* 双链表ADT */

struct rov_WaitQueue
{
    u32 status;
    rov_DoubleList_t wait_list;
};
typedef struct rov_WaitQueue* rov_WaitQueue_t; /* 等待队列ADT */

//TODO:基本内核对象
struct rov_Core
{
    u8 name[NITORI_CORE_NAME_MAX_LENGTH];
    u8 type;
    u8 flag;
    rov_DoubleList_t core_list; /* 内核对象链表 */
};
typedef struct rov_Core* rov_Core_t; /* 基本内核类 */

struct rov_Core_information
{
    rov_Core_Type type;
    rov_DoubleList_t object_list;
    u8 object_list_size;
};
typedef struct rov_Core_information rov_Core_information_t; /* 内核对象属性类 */







#endif