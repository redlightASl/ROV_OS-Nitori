#include "Core.h"

/**
 * @brief 初始化内核链表
 * @param  list             链表对象
 */
static ROV_INLINE void rov_List_init(rov_DoubleList_t* list)
{
    list->__next__ = list->__prev__ = list;
}

/**
 * @brief 在链表末尾插入链表节点
 * @param  list             原始链表
 * @param  node             新节点
 */
static ROV_INLINE void rov_List_insert_after(rov_DoubleList_t* list, rov_DoubleList_t* node)
{
    list->__next__->__prev__ = node;
    node->__next__ = list->__next__;
    list->__next__ = node;
    node->__prev__ = list;
}

/**
 * @brief 在链表前端插入链表节点
 * @param  list             原始链表
 * @param  node             新节点
 */
static ROV_INLINE void rov_List_insert_before(rov_DoubleList_t* list, rov_DoubleList_t* node)
{
    list->__prev__->__next__ = node;
    node->__prev__ = list->__prev__;
    list->__prev__ = node;
    node->__next__ = list;
}

/**
 * @brief 从链表末尾移除节点
 * @param  node             要操作的链表节点
 */
static ROV_INLINE void rov_List_remove(rov_DoubleList_t* node)
{
    node->__next__->__prev__ = node->__prev__;
    node->__prev__->__next__ = node->__next__;

    node->__next__ = node->__prev__ = node;
}

/**
 * @brief 链表查空
 * @param  list             要操作的链表
 * @return u8 1为空，0为非空
 */
static ROV_INLINE u8 rov_List_isempty(const rov_DoubleList_t* list)
{
    return list->__next__ == list;
}

/**
 * @brief 获取链表长度
 * @param list             要操作的链表
 */
static ROV_INLINE u8 rov_List_len(const rov_DoubleList_t* list)
{
    u8 len = 0;
    const rov_DoubleList_t* p = list;
    while (p->__next__ != list)
    {
        p = p->__next__;
        len++;
    }
    return len;
}











/**
 * @brief systick中断处理函数
 * @return WEAK void 无返回值
 */
// WEAK void SysTick_Handler(void)
// {
//     rov_interrupt_enter();
//     Systick_increase();
//     rov_interrupt_leave();
// }