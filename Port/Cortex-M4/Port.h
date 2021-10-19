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
#include <Setup.h>
#include <Defines.h>

#ifdef __cplusplus
extern  "C" {
#endif

#if /* ARMCC */ (  (defined ( __CC_ARM ) && defined ( __TARGET_FPU_VFP ))    \
    /* Clang */ || (defined ( __CLANG_ARM ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)) \
    /* IAR */   || (defined ( __ICCARM__ ) && defined ( __ARMVFP__ ))        \
    /* GNU */   || (defined ( __GNUC__ ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)) )
#define USE_FPU   1
#else
#define USE_FPU   0
#endif

#define NVIC_INT_CTRL       (0xE000ED04)      // 中断控制及状态寄存器
#define NVIC_PENDSVSET      (0x10000000)      // 触发软件中断的值
#define NVIC_SYSPRI2        (0xE000ED22)      // 系统优先级寄存器
#define NVIC_PENDSV_PRI     (0x000000FF)      // 配置优先级

#define VECTACTIVE_MASK     (0xFFUL)          // ICSR寄存器VECTACTIVE位掩码

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
#define SCB_MMAR        (*(volatile const unsigned *)0xE000ED34)        /* 内存管理错误地址寄存器 */
#define SCB_CFSR_MFSR   (*(volatile const unsigned char*)0xE000ED28)    /* 内存管理错误状态寄存器 */
#define SCB_BFAR        (*(volatile const unsigned *)0xE000ED38)        /* 总线错误地址寄存器 */
#define SCB_CFSR_BFSR   (*(volatile const unsigned char*)0xE000ED29)    /* 总线错误状态寄存器 */
#define SCB_AIRCR       (*(volatile unsigned long *)0xE000ED0C)         /* 复位控制寄存器 */
#define SCB_RESET_VALUE 0x05FA0004                                      /* 复位值,将它写到SCB_AIRCR可以引起CPU软复位 */
#define SCB_CFSR        (*(volatile const unsigned *)0xE000ED28)        /* 可配置错误状态寄存器 */
#define SCB_HFSR        (*(volatile const unsigned *)0xE000ED2C)        /* HardFault状态寄存器 */
#define SCB_CFSR_UFSR   (*(volatile const unsigned short*)0xE000ED2A)   /* 使用错误状态寄存器 */


//XXX：从rtt抄的移植结构，包括栈初始化、异常处理和cpu软件复位
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

    struct fpu_exception_stack_frame
    {
        u32 r0;
        u32 r1;
        u32 r2;
        u32 r3;
        u32 r12;
        u32 lr;
        u32 pc;
        u32 psr;

#if USING_FPU
        /* FPU 寄存器 */
        u32 S0;
        u32 S1;
        u32 S2;
        u32 S3;
        u32 S4;
        u32 S5;
        u32 S6;
        u32 S7;
        u32 S8;
        u32 S9;
        u32 S10;
        u32 S11;
        u32 S12;
        u32 S13;
        u32 S14;
        u32 S15;
        u32 FPSCR;
        u32 NO_NAME;
#endif
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

    struct fpu_task_stack_frame
    {
        u32 flag;

        /* r4 ~ r11 寄存器 */
        u32 r4;
        u32 r5;
        u32 r6;
        u32 r7;
        u32 r8;
        u32 r9;
        u32 r10;
        u32 r11;

#if USING_FPU
        /* FPU寄存器 s16 ~ s31 */
        u32 s16;
        u32 s17;
        u32 s18;
        u32 s19;
        u32 s20;
        u32 s21;
        u32 s22;
        u32 s23;
        u32 s24;
        u32 s25;
        u32 s26;
        u32 s27;
        u32 s28;
        u32 s29;
        u32 s30;
        u32 s31;
#endif

        struct fpu_exception_stack_frame exception_stack_frame;
    };

    struct exception_info
    {
        u32 exc_return;
        struct task_stack_frame stack_frame;
    };

    u8* rov_thread_stack_init(void* task_entry, void* parameter, u8* stack_addr, void* texit)
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

#if USING_FPU
        stack_frame->flag = 0;
#endif /* USE_FPU */

        /* return task's current stack address */
        return stack;
    }

    void rov_exception_install(u8(*exception_handle)(void* context))
    {
        rov_exception_hook = exception_handle;
    }

    /**
     * shutdown CPU
     */
    RT_WEAK void rt_hw_cpu_shutdown(void)
    {
        rt_kprintf("shutdown...\n");

        RT_ASSERT(0);
    }

    ROV_WEAK void rov_cpu_reset(void)
    {
        SCB_AIRCR = SCB_RESET_VALUE;
    }









































    /* 全局中断控制 */
    //TODO:这里预计使用内嵌汇编而不是独立的.S文件s
#define DISABLE_INTERRUPTS() rov_interrupt_disable() //DEBUG:在.S中有定义
#define ENABLE_INTERRUPTS() rov_interrupt_enable() //DEBUG:在.S中有定义
    static ROV_ALWAYS_INLINE void rov_interrupt_disable(void)
    {
        __asm
        {
            /* freertos思路：把BASEPRI提高到最大系统调用优先级来防止中断发生 */
            MRS basepri, ulNewBASEPRI
            DSB
            ISB
            /* rtt思路：把r0设置为最高优先级 */
            MRS     r0, PRIMASK
            CPSID   I
            BX      LR
        }
    }

    static ROV_ALWAYS_INLINE void rov_interrupt_enable(u32 basepri)
    {
        __asm
        {
            /* freertos思路：降低BASEPRI来使能其他中断 */
            MSR basepri, ulBASEPRI
            /* rtt思路：返回r0 */
            MSR     PRIMASK, r0
            BX      LR
        }
    }

#define ENTER_CRITICAL() rov_enter_critical()
#define EXIT_CRITICAL() rov_exit_critical()
    static ROV_ALWAYS_INLINE void rov_enter_critical(void)
    {
        /* freertos思路：关闭中断，再设置嵌套中断层数 */
        DISABLE_INTERRUPTS();
        interrupt_nest++;
        if (interrupt_nest == 1)
        {
            ROV_ASSERT((NVIC_INT_CTRL & VECTACTIVE_MASK) == 0);
        }
        /* rtt思路：返回r0 */
        rov_BaseType level;
        level = rov_interrupt_enable();
        interrupt_nest++; //指示嵌套中断层数的全局变量
        rov_interrupt_disable(level);
    }

    static ROV_ALWAYS_INLINE void rov_exit_critical(void)
    {
        /* freertos思路：降低BASEPRI来使能其他中断 */
        ROV_ASSERT(interrupt_nest);
        interrupt_nest--;
        if (interrupt_nest == 0)
        {
            ENABLE_INTERRUPTS();
        }
        /* rtt思路：返回r0 */
        rov_BaseType level;
        level = rov_interrupt_disable();
        interrupt_nest--; //指示嵌套中断层数的全局变量
        rov_interrupt_enable(level);
    }




    /* PendSV中断服务函数 */
    //TODO:这里预计使用内嵌汇编而不是独立的.S文件


    //XXX：freertos的PendSV中断服务函数

    /*
     * Exception handlers.
     */
    void xPortPendSVHandler(void);
    __asm void xPortPendSVHandler(void)
    {
        extern uxCriticalNesting;
        extern pxCurrentTCB;
        extern vTaskSwitchContext;

        /* *INDENT-OFF* */
        PRESERVE8

            mrs r0, psp
            isb
            /* Get the location of the current TCB. */
            ldr r3, =pxCurrentTCB
            ldr r2, [r3]

            /* Is the task using the FPU context?  If so, push high vfp registers. */
            tst r14, #0x10
            it eq
            vstmdbeq r0!, { s16 - s31 }

            /* Save the core registers. */
            stmdb r0!, { r4 - r11, r14 }

            /* Save the new top of stack into the first member of the TCB. */
            str r0, [r2]

            stmdb sp!, { r0, r3 }
            mov r0, # configMAX_SYSCALL_INTERRUPT_PRIORITY
            msr basepri, r0
            dsb
            isb
            bl vTaskSwitchContext
            mov r0, # 0
            msr basepri, r0
            ldmia sp!, { r0, r3 }

            /* The first item in pxCurrentTCB is the task top of stack. */
            ldr r1, [r3]
            ldr r0, [r1]

            /* Pop the core registers. */
            ldmia r0!, { r4 - r11, r14 }

            /* Is the task using the FPU context?  If so, pop the high vfp registers
             * too. */
            tst r14, # 0x10
            it eq
            vldmiaeq r0!, { s16 - s31 }

            msr psp, r0
            isb
#ifdef WORKAROUND_PMU_CM001 /* XMC4000 specific errata */
#if WORKAROUND_PMU_CM001 == 1
            push{ r14 }
            pop{ pc }
            nop
#endif
#endif

            bx r14
            /* *INDENT-ON* */
    }


    //XXX：RTT的PendSV中断服务函数
    /**
     * @brief PendSV 线程切换函数
     */
    static ROV_INLINE void PendSV_Handler(void)
    {
        __asm
        {
            IMPORT  blockPtr

            // 加载寄存器存储地址
            LDR     R0, =blockPtr
            LDR     R0, [R0]
            LDR     R0, [R0]

            // 保存寄存器
            STMDB   R0!, { R4 - R11 }

            // 将最后的地址写入到blockPtr中
            LDR     R1, =blockPtr
            LDR     R1, [R1]
            STR     R0, [R1]

            // // 修改部分寄存器，用于测试
            // ADD R4, R4, #1
            // ADD R5, R5, #1

            // 恢复寄存器
            LDMIA   R0!, { R4 - R11 }

            // 异常返回
            BX      LR











            /* XXX:直接照抄RT-Thread的PendSV中断服务函数
 * r0 --> switch from thread stack
 * r1 --> switch to thread stack
 * psr, pc, lr, r12, r3, r2, r1, r0 are pushed into [from] stack
 */
 .global PendSV_Handler
 .type PendSV_Handler, % function
 PendSV_Handler :
            /* 关中断保护线程切换 */
            MRS r2, PRIMASK
                CPSID   I

                /* 获取rov_thread_switch_interrupt_flag标志位 */
                LDR r0, =rov_thread_switch_interrupt_flag
                LDR r1, [r0]
                CBZ r1, pendsv_exit

                /* 清除rov_thread_switch_interrupt_flag标志位 */
                MOV r1, #0x00
                STR r1, [r0]

                LDR r0, =rov_interrupt_from_thread
                LDR r1, [r0]
                CBZ r1, switch_to_thread

                MRS r1, psp

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
                TST     lr, #0x10
                VSTMDBEQ r1!, { d8 - d15 }
#endif

                STMFD   r1!, { r4 - r11 }

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
                MOV     r4, #0x00

                TST     lr, #0x10
                MOVEQ   r4, #0x01

                STMFD   r1!, { r4 }
#endif

                LDR r0, [r0]
                STR r1, [r0]

                switch_to_thread:
            LDR r1, =rov_interrupt_to_thread
                LDR r1, [r1]
                LDR r1, [r1]

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
                LDMFD   r1!, { r3 }
#endif

                LDMFD   r1!, { r4 - r11 }

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
                CMP     r3, #0
                VLDMIANE  r1!, { d8 - d15 }
#endif

                MSR psp, r1

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
                ORR     lr, lr, #0x10
                CMP     r3, #0
                BICNE   lr, lr, #0x10
#endif

            pendsv_exit:
            MSR PRIMASK, r2

                ORR lr, lr, #0x04
                BX  lr
        }
    }




























//XXX:从FreeRTOS照抄的systick函数
    void vPortSetupTimerInterrupt(void);
    __weak void vPortSetupTimerInterrupt(void)
    {
        /* Calculate the constants required to configure the tick interrupt. */
#if ( configUSE_TICKLESS_IDLE == 1 )
        {
            ulTimerCountsForOneTick = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);
            xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
            ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);
        }
#endif /* configUSE_TICKLESS_IDLE */

        /* Stop and clear the SysTick. */
        portNVIC_SYSTICK_CTRL_REG = 0UL;
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

        /* Configure SysTick to interrupt at the requested rate. */
        portNVIC_SYSTICK_LOAD_REG = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
        portNVIC_SYSTICK_CTRL_REG = (portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT);
    }



    
    void xPortSysTickHandler(void);
    void xPortSysTickHandler(void)
    {
        vPortRaiseBASEPRI();
        {
            /* Increment the RTOS tick. */
            if (xTaskIncrementTick() != pdFALSE)
            {
                /* A context switch is required.  Context switching is performed in
                 * the PendSV interrupt.  Pend the PendSV interrupt. */
                portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
            }
        }

        vPortClearBASEPRIFromISR();
    }


































#ifdef __cplusplus
}
#endif

#endif