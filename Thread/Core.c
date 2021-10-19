#include "Core.h"

/**
 * @brief 初始化内核链表
 * @param  list             链表对象
 */
static ROV_INLINE void rov_List_init(rov_DoubleList_t* list)
{
    list->next = list->prev = list;
}

/**
 * @brief 在链表末尾插入链表节点
 * @param  list             原始链表
 * @param  node             新节点
 */
static ROV_INLINE void rov_List_insert_after(rov_DoubleList_t* list, rov_DoubleList_t* node)
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
static ROV_INLINE void rov_List_insert_before(rov_DoubleList_t* list, rov_DoubleList_t* node)
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
static ROV_INLINE void rov_List_remove(rov_DoubleList_t* node)
{
    node->next->prev = node->prev;
    node->prev->next = node->next;

    node->next = node->prev = node;
}

/**
 * @brief 链表查空
 * @param  list             要操作的链表
 * @return u8 1为空，0为非空
 */
static ROV_INLINE u8 rov_List_isempty(const rov_DoubleList_t* list)
{
    return list->next == list;
}

/**
 * @brief 获取链表长度
 * @param list             要操作的链表
 */
static ROV_INLINE u8 rov_List_len(const rov_DoubleList_t* list)
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











//XXX:照抄rtt
WEAK void SysTick_Handler(void)
{
    rov_interrupt_enter();
    Systick_increase();
    rov_interrupt_leave();
}