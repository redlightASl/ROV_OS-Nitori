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



 //XXX:没什么用的单链表
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





struct rov_Thread
{
    u32 thread_id; //线程id
    char name[MAX_LENGTH_OF_CORE_NAME];
    u8  type;
    u8  flags;

    void (*thread_handler)(void); //线程句柄
    // rov_Thread_Status status; //线程状态
    u8 status; //线程状态

    /* 链表 */
    rov_DoubleList_t core_list; //内核对象链表
    rov_DoubleList_t thread_list; //线程链表

    /* 栈指针和入口函数 */
    void* sp;
    void* entry;
    void* parameter;
    void* stack_addr;
    u32 stack_size;

    /* 时间片 */
    u32 thread_init_time; //线程初始化时间片
    u32 thread_remaining_time; //线程剩余时间片

    /* 优先级 */
    u8 current_priority; //当前优先级
    u8 init_priority; //初始优先级
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
    const rov_DoubleList_t* p = list;
    while (p->next != list)
    {
        p = p->next;
        len++;
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