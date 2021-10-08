#ifndef __ROV_INTERRUPT_H
#define __ROV_INTERRUPT_H














//TODO:这里使用自己实现的中断控制函数
void rt_hw_interrupt_init(void);
void rt_hw_interrupt_mask(int vector);
void rt_hw_interrupt_umask(int vector);
rt_isr_handler_t rt_hw_interrupt_install(int vector, rt_isr_handler_t handler, void* param, const char* name);
rt_base_t rt_hw_interrupt_disable(void);
void rt_hw_interrupt_enable(rt_base_t level);




#endif


