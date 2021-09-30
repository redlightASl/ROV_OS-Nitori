/**
 * @file Port.h
 * @brief Nitori-OS移植配置
 * @author RedlightASl (dddbbbdd@foxmail.com)
 * @version 1.0
 * @date 2021-09-30
 *
 * @copyright Copyright (c) 2021  RedlightASl
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-09-30 <td>1.0     <td>wangh     <td>Content
 * </table>
 */
#ifndef __ROV_PORT_H
#define __ROV_PORT_H
#include <Defines.h>
#include <HardwareAccelerate.h>

 /* 异常服务函数表 */
 //TODO:这里预计使用内嵌汇编而不是独立的.S文件
u32 rov_interrupt_from_thread;
u32 rov_interrupt_to_thread;
u32 rov_thread_switch_interrupt_flag;
/* 异常钩子函数 */
static u8(*rov_exception_hook)(void* context) = 0;

struct exception_stack_frame
{
    u32 r0;
    u32 r1;
    u32 r2;
    u32 r3;
    u32 r12;
    u32 lr;
    u32 pc;
    u32 psr;
};

struct task_stack_frame
{
#ifdef USING_FPU
    u32 flag;
#endif
    /* r4 ~ r11 寄存器 */
    u32 r4;
    u32 r5;
    u32 r6;
    u32 r7;
    u32 r8;
    u32 r9;
    u32 r10;
    u32 r11;

    struct exception_stack_frame exception_stack_frame;
};

//TODO:这里使用自己实现的中断控制函数
void rt_hw_interrupt_init(void);
void rt_hw_interrupt_mask(int vector);
void rt_hw_interrupt_umask(int vector);
rt_isr_handler_t rt_hw_interrupt_install(int vector, rt_isr_handler_t handler, void* param, const char* name);
rt_base_t rt_hw_interrupt_disable(void);
void rt_hw_interrupt_enable(rt_base_t level);

/* ARM异常号 */
#define  OS_CPU_ARM_EXCEPT_RESET                                                                    0x00u
#define  OS_CPU_ARM_EXCEPT_UNDEF_INSTR                                                              0x01u
#define  OS_CPU_ARM_EXCEPT_SWI                                                                      0x02u
#define  OS_CPU_ARM_EXCEPT_PREFETCH_ABORT                                                           0x03u
#define  OS_CPU_ARM_EXCEPT_DATA_ABORT                                                               0x04u
#define  OS_CPU_ARM_EXCEPT_ADDR_ABORT                                                               0x05u
#define  OS_CPU_ARM_EXCEPT_IRQ                                                                      0x06u
#define  OS_CPU_ARM_EXCEPT_FIQ                                                                      0x07u
#define  OS_CPU_ARM_EXCEPT_NBR                                                                      0x08u
/* ARM异常向量地址 */
#define  OS_CPU_ARM_EXCEPT_RESET_VECT_ADDR              (OS_CPU_ARM_EXCEPT_RESET          * 0x04u + 0x00u)
#define  OS_CPU_ARM_EXCEPT_UNDEF_INSTR_VECT_ADDR        (OS_CPU_ARM_EXCEPT_UNDEF_INSTR    * 0x04u + 0x00u)
#define  OS_CPU_ARM_EXCEPT_SWI_VECT_ADDR                (OS_CPU_ARM_EXCEPT_SWI            * 0x04u + 0x00u)
#define  OS_CPU_ARM_EXCEPT_PREFETCH_ABORT_VECT_ADDR     (OS_CPU_ARM_EXCEPT_PREFETCH_ABORT * 0x04u + 0x00u)
#define  OS_CPU_ARM_EXCEPT_DATA_ABORT_VECT_ADDR         (OS_CPU_ARM_EXCEPT_DATA_ABORT     * 0x04u + 0x00u)
#define  OS_CPU_ARM_EXCEPT_ADDR_ABORT_VECT_ADDR         (OS_CPU_ARM_EXCEPT_ADDR_ABORT     * 0x04u + 0x00u)
#define  OS_CPU_ARM_EXCEPT_IRQ_VECT_ADDR                (OS_CPU_ARM_EXCEPT_IRQ            * 0x04u + 0x00u)
#define  OS_CPU_ARM_EXCEPT_FIQ_VECT_ADDR                (OS_CPU_ARM_EXCEPT_FIQ            * 0x04u + 0x00u)
/* ARM异常服务函数地址 */
#define  OS_CPU_ARM_EXCEPT_RESET_HANDLER_ADDR           (OS_CPU_ARM_EXCEPT_RESET          * 0x04u + 0x20u)
#define  OS_CPU_ARM_EXCEPT_UNDEF_INSTR_HANDLER_ADDR     (OS_CPU_ARM_EXCEPT_UNDEF_INSTR    * 0x04u + 0x20u)
#define  OS_CPU_ARM_EXCEPT_SWI_HANDLER_ADDR             (OS_CPU_ARM_EXCEPT_SWI            * 0x04u + 0x20u)
#define  OS_CPU_ARM_EXCEPT_PREFETCH_ABORT_HANDLER_ADDR  (OS_CPU_ARM_EXCEPT_PREFETCH_ABORT * 0x04u + 0x20u)
#define  OS_CPU_ARM_EXCEPT_DATA_ABORT_HANDLER_ADDR      (OS_CPU_ARM_EXCEPT_DATA_ABORT     * 0x04u + 0x20u)
#define  OS_CPU_ARM_EXCEPT_ADDR_ABORT_HANDLER_ADDR      (OS_CPU_ARM_EXCEPT_ADDR_ABORT     * 0x04u + 0x20u)
#define  OS_CPU_ARM_EXCEPT_IRQ_HANDLER_ADDR             (OS_CPU_ARM_EXCEPT_IRQ            * 0x04u + 0x20u)
#define  OS_CPU_ARM_EXCEPT_FIQ_HANDLER_ADDR             (OS_CPU_ARM_EXCEPT_FIQ            * 0x04u + 0x20u)

/* CPU SCB寄存器地址 */
#define SCB_CFSR        (*(volatile const unsigned *)0xE000ED28)        /* Configurable Fault Status Register */
#define SCB_HFSR        (*(volatile const unsigned *)0xE000ED2C)        /* HardFault Status Register */
#define SCB_MMAR        (*(volatile const unsigned *)0xE000ED34)        /* MemManage Fault Address register */
#define SCB_BFAR        (*(volatile const unsigned *)0xE000ED38)        /* Bus Fault Address Register */
#define SCB_AIRCR       (*(volatile unsigned long *)0xE000ED0C)         /* Reset control Address Register */
#define SCB_RESET_VALUE 0x05FA0004                                      /* Reset value, write to SCB_AIRCR can reset cpu */
#define SCB_CFSR_MFSR   (*(volatile const unsigned char*)0xE000ED28)    /* Memory-management Fault Status Register */
#define SCB_CFSR_BFSR   (*(volatile const unsigned char*)0xE000ED29)    /* Bus Fault Status Register */
#define SCB_CFSR_UFSR   (*(volatile const unsigned short*)0xE000ED2A)   /* Usage Fault Status Register */

u8* rov_thread_stack_init(void* tentry, void* parameter, u8* stack_addr, void* texit)
{
    struct task_stack_frame* stack_frame;
    u8* stack;

    stack = stack_addr + sizeof(u32);
    stack = (u8*)ROV_ALIGN_DOWN((u32)stack, 8);
    stack -= sizeof(struct task_stack_frame);

    stack_frame = (struct task_stack_frame*)stack;

    /* init all register */
    for (unsigned long i = 0; i < sizeof(struct task_stack_frame) / sizeof(u32); i++)
    {
        ((u32*)stack_frame)[i] = 0xdeadbeef;
    }

    stack_frame->exception_stack_frame.r0 = (unsigned long)parameter; /* r0 : argument */
    stack_frame->exception_stack_frame.r1 = 0;                        /* r1 */
    stack_frame->exception_stack_frame.r2 = 0;                        /* r2 */
    stack_frame->exception_stack_frame.r3 = 0;                        /* r3 */
    stack_frame->exception_stack_frame.r12 = 0;                        /* r12 */
    stack_frame->exception_stack_frame.lr = (unsigned long)texit;     /* lr */
    stack_frame->exception_stack_frame.pc = (unsigned long)tentry;    /* entry point, pc */
    stack_frame->exception_stack_frame.psr = 0x01000000L;              /* PSR */

#if USE_FPU
    stack_frame->flag = 0;
#endif /* USE_FPU */

    /* return task's current stack address */
    return stack;
}

void rov_exception_install(u8(*exception_handle)(void* context))
{
    rov_exception_hook = exception_handle;
}

ROV_WEAK void rov_cpu_reset(void)
{
    SCB_AIRCR = SCB_RESET_VALUE;
}

// u32 CrcCalculate(u8* CacString, u32 CacStringSize);

#endif