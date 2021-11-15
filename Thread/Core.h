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
#include "Setup.h"


//DONE:链表部分已经完成
struct rov_SingleList
{
    struct rov_SingleList* __next__;
};
typedef struct rov_SingleList* rov_SingleList_t; /* 单链表ADT */

struct rov_DoubleList
{
    struct rov_DoubleList* __prev__;
    struct rov_DoubleList* __next__;
};
typedef struct rov_DoubleList rov_DoubleList_t; /* 双链表ADT */

struct rov_WaitQueue
{
    u8 flag; //队列状态
    u8 wait_number; //队列里面处于等待状态对象的数量 
    rov_DoubleList_t wait_list;
};
typedef struct rov_WaitQueue* rov_WaitQueue_t; /* 等待队列ADT */

//TODO:基本内核对象
struct rov_Core
{
    u8 name[NITORI_CORE_NAME_MAX_LENGTH]; //内核对象名
    u8 type; //内核对象类型
    u8 flag; //内核对象状态
    rov_DoubleList_t core_list; /* 内核对象链表 */
};
typedef struct rov_Core* rov_Core_t; /* 基本内核类 */

struct rov_Core_information
{
    u8 type; //内核对象类型
    u8 object_list_size; //内核对象链表的大小
    rov_DoubleList_t object_list; //内核对象链表
};
typedef struct rov_Core_information* rov_Core_information_t; /* 内核对象属性类 */







#endif