/**
 * @file Thread.h
 * @brief Nitori内核部分-线程时间片轮转调度
 * @author RedlightASl (dddbbbdd@foxmail.com)
 * @version 1.0
 * @date 2021-09-18
 *
 * @copyright Copyright (c) 2021  RedlightASl
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-09-18 <td>1.0     <td>wangh     <td>Content
 * </table>
 */
#ifndef __ROV_THREAD_H
#define __ROV_THREAD_H
#include <Defines.h>
#include <Setup.h>
#include <Core.h>
 // #ifndef NITORI_ONLY_THREAD
 // #include <Core.h>
 // #else
 // struct rov_DoubleList
 // {
 //     struct rov_DoubleList* prev;
 //     struct rov_DoubleList* next;
 // };
 // typedef struct rov_DoubleList rov_DoubleList_t; /* 双链表ADT */

 // static ROV_INLINE void rov_List_init(rov_DoubleList_t* list)
 // {
 //     list->next = list->prev = list;
 // }

 // static ROV_INLINE void rov_List_insert_after(rov_DoubleList_t* list, rov_DoubleList_t* node)
 // {
 //     list->next->prev = node;
 //     node->next = list->next;
 //     list->next = node;
 //     node->prev = list;
 // }

 // static ROV_INLINE void rov_List_insert_before(rov_DoubleList_t* list, rov_DoubleList_t* node)
 // {
 //     list->prev->next = node;
 //     node->prev = list->prev;
 //     list->prev = node;
 //     node->next = list;
 // }

 // static ROV_INLINE void rov_List_remove(rov_DoubleList_t* node)
 // {
 //     node->next->prev = node->prev;
 //     node->prev->next = node->next;

 //     node->next = node->prev = node;
 // }

 // static ROV_INLINE u8 rov_List_isempty(const rov_DoubleList_t* list)
 // {
 //     return list->next == list;
 // }

 // static ROV_INLINE u8 rov_List_len(const rov_DoubleList_t* list)
 // {
 //     u8 len = 0;
 //     const rov_DoubleList_t* p = list;
 //     while (p->next != list)
 //     {
 //         p = p->next;
 //         len++;
 //     }
 //     return len;
 // }
 // #endif

struct rov_Thread
{
    u32 thread_id; //线程id
    char name[NITORI_CORE_NAME_MAX_LENGTH];
    u8  type;
    u8  flags;

    void (*thread_handler)(void); //线程句柄
    u8 status; //线程状态

    /* 优先级 */
    u8 current_priority; //当前优先级
    u8 init_priority; //初始优先级

    /* 链表 */
    rov_DoubleList_t core_list; //内核对象链表
    rov_DoubleList_t thread_list; //线程链表

    /* 栈指针和入口函数 */
    void* ThreadStack;
    void* entry;
    void* parameter;
    void* stack_addr;
    u32 stack_size;

    /* 时间片 */
    void (*timeout_function)(void* parameter); //超时函数
    void* parameter; //超时函数参数
    u32 thread_init_time; //线程初始化时间片
    u32 thread_remaining_time; //线程剩余时间片
};
typedef struct rov_Thread* rov_Thread_t; /* 线程类 */



























// void Thread_exec(void);
// void Thread_tick(void);
// void Thread_init(rov_Thread_t thread, void (*thread_handler)(void), u32 id, u32 thread_time_length);
// void Thread_add(rov_Thread_t thread);
// void Thread_delete(rov_Thread_t thread);


#endif