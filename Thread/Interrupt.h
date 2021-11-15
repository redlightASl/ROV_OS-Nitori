#ifndef __ROV_INTERRUPT_H
#define __ROV_INTERRUPT_H
#include <Defines.h>
#include <Port.h>

typedef void (*rov_ISR_t)(u32 vector, void *param);









//TODO:这里使用自己实现的中断控制函数
void rov_Interrupt_init(void);
void rov_Interrupt_mask(u32 vector);
void rov_Interrupt_unmask(u32 vector);
void rov_Interrupt_enter(u32 level);
u32 rov_Interrupt_leave(void);
rov_ISR_t rov_Interrupt_install(u32 vector, rov_ISR_t isr_handler, void* param, const char* name);





#endif


