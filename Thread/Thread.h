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
#include "Defines.h"
#include "Setup.h"

typedef enum
{
    THREAD_STATUS_RUN,
    THREAD_STATUS_SHUTDOWN
} rov_Thread_Status;

struct rov_Thread
{
    u32 thread_id; //线程id
    void (*thread_handler)(void); //线程句柄
    rov_Thread_Status status; //线程状态
    u32 thread_timer; //线程定时器
    u32 thread_time_length; //线程时间片
    //线程链表
};
typedef struct rov_Thread* rov_Thread_t;

void Thread_exec(void);
void Thread_tick(void);
void Thread_init(rov_Thread_t thread, void (*thread_handler)(void), u32 id, u32 thread_time_length);
void Thread_add(rov_Thread_t thread);
void Thread_delete(rov_Thread_t thread);


#endif