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




struct rov_SingleList
{
    struct rov_SingleList* next;
};
typedef struct rov_SingleList rov_SingleList_t; /* 单链表类 */

struct rov_DoubleList
{
    struct rov_DoubleList* prev;
    struct rov_DoubleList* next;
};
typedef struct rov_DoubleList rov_DoubleList_t; /* 双链表类 */

struct rov_Core
{
    u8 name[MAX_LENGTH_OF_CORE_NAME];
    u8 type;
    u8 flag;
    rov_DoubleList_t list; //线程链表
};
typedef struct rov_Core* rov_Core_t; /* 基本内核类 */



struct rt_object_information
{
    enum rt_object_class_type type;                     /**< object class type */
    rt_list_t                 object_list;              /**< object list */
    rt_size_t                 object_size;              /**< object size */
};




struct rt_thread
{
    /* rt object */
    char        name[RT_NAME_MAX];                      /**< the name of thread */
    rt_uint8_t  type;                                   /**< type of object */
    rt_uint8_t  flags;                                  /**< thread's flags */

#ifdef RT_USING_MODULE
    void       *module_id;                              /**< id of application module */
#endif

    rt_list_t   list;                                   /**< the object list */
    rt_list_t   tlist;                                  /**< the thread list */

    /* stack point and entry */
    void       *sp;                                     /**< stack point */
    void       *entry;                                  /**< entry */
    void       *parameter;                              /**< parameter */
    void       *stack_addr;                             /**< stack address */
    rt_uint32_t stack_size;                             /**< stack size */

    /* error code */
    rt_err_t    error;                                  /**< error code */

    rt_uint8_t  stat;                                   /**< thread status */

#ifdef RT_USING_SMP
    rt_uint8_t  bind_cpu;                               /**< thread is bind to cpu */
    rt_uint8_t  oncpu;                                  /**< process on cpu` */

    rt_uint16_t scheduler_lock_nest;                    /**< scheduler lock count */
    rt_uint16_t cpus_lock_nest;                         /**< cpus lock count */
    rt_uint16_t critical_lock_nest;                     /**< critical lock count */
#endif /*RT_USING_SMP*/

    /* priority */
    rt_uint8_t  current_priority;                       /**< current priority */
    rt_uint8_t  init_priority;                          /**< initialized priority */
#if RT_THREAD_PRIORITY_MAX > 32
    rt_uint8_t  number;
    rt_uint8_t  high_mask;
#endif
    rt_uint32_t number_mask;

#if defined(RT_USING_EVENT)
    /* thread event */
    rt_uint32_t event_set;
    rt_uint8_t  event_info;
#endif

#if defined(RT_USING_SIGNALS)
    rt_sigset_t     sig_pending;                        /**< the pending signals */
    rt_sigset_t     sig_mask;                           /**< the mask bits of signal */

#ifndef RT_USING_SMP
    void            *sig_ret;                           /**< the return stack pointer from signal */
#endif
    rt_sighandler_t *sig_vectors;                       /**< vectors of signal handler */
    void            *si_list;                           /**< the signal infor list */
#endif

    rt_ubase_t  init_tick;                              /**< thread's initialized tick */
    rt_ubase_t  remaining_tick;                         /**< remaining tick */

#ifdef RT_USING_CPU_USAGE
    rt_uint64_t  duration_tick;                          /**< cpu usage tick */
#endif

    struct rt_timer thread_timer;                       /**< built-in thread timer */

    void (*cleanup)(struct rt_thread *tid);             /**< cleanup function when thread exit */

    /* light weight process if present */
#ifdef RT_USING_LWP
    void        *lwp;
#endif

    rt_ubase_t user_data;                             /**< private user data beyond this thread */
};
typedef struct rt_thread *rt_thread_t;

struct rov_Thread
{
    u32 thread_id; //线程id
    void (*thread_handler)(void); //线程句柄
    rov_Thread_Status status; //线程状态
    u32 thread_timer; //线程定时器
    u32 thread_time_length; //线程时间片
    rov_List_t list; //线程链表
};
typedef struct rov_Thread* rov_Thread_t; /* 线程内核类 */











/**
 * @brief 初始化内核链表
 * @param  list             链表对象
 */
static inline void rov_List_init(rov_DoubleList_t* list)
{
    list->next = list->prev = list;
}

/**
 * @brief 在链表末尾插入链表节点
 * @param  list             原始链表
 * @param  node             新节点
 */
static inline void rov_List_insert_after(rov_DoubleList_t* list, rov_DoubleList_t* node)
{
    list->next->prev = node;
    node->next = list->next;

    list->next = node;
    node->prev = list;
}

/**
 * @brief 在链表前端插入链表节点
 * @param  list             原始链表
 * @param  node             新节点
 */
static inline void rov_List_insert_before(rov_DoubleList_t* list, rov_DoubleList_t* node)
{
    list->prev->next = node;
    node->prev = list->prev;

    list->prev = node;
    node->next = list;
}

/**
 * @brief 从链表末尾移除节点
 * @param  node             要操作的链表节点
 */
static inline void rov_List_remove(rov_DoubleList_t* node)
{
    node->next->prev = node->prev;
    node->prev->next = node->next;

    node->next = node->prev = node;
}

/**
 * @brief 链表查空
 * @param  list             链表
 * @return u8 1为空，0为非空
 */
static inline u8 rov_List_isempty(const rov_DoubleList_t* list)
{
    return list->next == list;
}

/**
 * @brief 获取链表长度
 * @param list             要操作的链表
 */
static inline u8 rov_List_len(const rov_DoubleList_t* list)
{
    u8 len = 0;
    const rov_DoubleList_t *p = list;
    while (p->next != list)
    {
        p = p->next;
        len ++;
    }
    return len;
}





/**
 * @brief get the struct for this entry
 * @param node the entry point
 * @param type the type of structure
 * @param member the name of list in structure
 */
#define rt_list_entry(node, type, member) \
    rt_container_of(node, type, member)

/**
 * rt_list_for_each - iterate over a list
 * @pos:    the rt_list_t * to use as a loop cursor.
 * @head:   the head for your list.
 */
#define rt_list_for_each(pos, head) \
    for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * rt_list_for_each_safe - iterate over a list safe against removal of list entry
 * @pos:    the rt_list_t * to use as a loop cursor.
 * @n:      another rt_list_t * to use as temporary storage
 * @head:   the head for your list.
 */
#define rt_list_for_each_safe(pos, n, head) \
    for (pos = (head)->next, n = pos->next; pos != (head); \
        pos = n, n = pos->next)

/**
 * rt_list_for_each_entry  -   iterate over list of given type
 * @pos:    the type * to use as a loop cursor.
 * @head:   the head for your list.
 * @member: the name of the list_struct within the struct.
 */
#define rt_list_for_each_entry(pos, head, member) \
    for (pos = rt_list_entry((head)->next, typeof(*pos), member); \
         &pos->member != (head); \
         pos = rt_list_entry(pos->member.next, typeof(*pos), member))

/**
 * rt_list_for_each_entry_safe - iterate over list of given type safe against removal of list entry
 * @pos:    the type * to use as a loop cursor.
 * @n:      another type * to use as temporary storage
 * @head:   the head for your list.
 * @member: the name of the list_struct within the struct.
 */
#define rt_list_for_each_entry_safe(pos, n, head, member) \
    for (pos = rt_list_entry((head)->next, typeof(*pos), member), \
         n = rt_list_entry(pos->member.next, typeof(*pos), member); \
         &pos->member != (head); \
         pos = n, n = rt_list_entry(n->member.next, typeof(*n), member))

/**
 * rt_list_first_entry - get the first element from a list
 * @ptr:    the list head to take the element from.
 * @type:   the type of the struct this is embedded in.
 * @member: the name of the list_struct within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define rt_list_first_entry(ptr, type, member) \
    rt_list_entry((ptr)->next, type, member)











void Thread_exec(void);
void Thread_tick(void);
void Thread_init(rov_Thread_t thread, void (*thread_handler)(void), u32 id, u32 thread_time_length);
void Thread_add(rov_Thread_t thread);
void Thread_delete(rov_Thread_t thread);


#endif