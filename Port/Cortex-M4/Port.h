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
#include "Defines.h"
#include "Setup.h"

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

    /* 系统控制寄存器 */
#define NVIC_INT_CTRL       (*((volatile uint32_t *)0xE000ED04))        // 中断控制及状态寄存器
#define NVIC_PENDSVSET      (*((volatile uint32_t *)0x10000000))        // 触发软件中断的值
#define NVIC_SYSPRI2        (*((volatile uint32_t *)0xE000ED22))        // 系统优先级寄存器
#define NVIC_PENDSV_PRI     (*((volatile uint32_t *)0x000000FF))        // 配置优先级
#define VECTACTIVE_MASK     (0xFFul)                                    // ICSR寄存器VECTACTIVE位掩码
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
#define SCB_MMAR        (*(volatile const unsigned *)       0xE000ED34)     /* 内存管理错误地址寄存器 */
#define SCB_CFSR_MFSR   (*(volatile const unsigned char*)   0xE000ED28)     /* 内存管理错误状态寄存器 */
#define SCB_BFAR        (*(volatile const unsigned *)       0xE000ED38)     /* 总线错误地址寄存器 */
#define SCB_CFSR_BFSR   (*(volatile const unsigned char*)   0xE000ED29)     /* 总线错误状态寄存器 */
#define SCB_AIRCR       (*(volatile unsigned long *)        0xE000ED0C)     /* 复位控制寄存器 */
#define SCB_RESET_VALUE                                     0x05FA0004      /* 复位值，将它写到SCB_AIRCR可以引起CPU软复位 */
#define SCB_CFSR        (*(volatile const unsigned *)       0xE000ED28)     /* 可配置错误状态寄存器 */
#define SCB_HFSR        (*(volatile const unsigned *)       0xE000ED2C)     /* HardFault状态寄存器 */
#define SCB_CFSR_UFSR   (*(volatile const unsigned short*)  0xE000ED2A)     /* 使用错误状态寄存器 */

/* 全局中断控制 */
#define DISABLE_INTERRUPTS()                    rov_interrupt_disable()
#define ENABLE_INTERRUPTS(basepri)              rov_interrupt_enable(basepri)

/* 断言 */
// #define ROV_ASSERT(x)                           if((0) == (x)) {DISABLE_INTERRUPTS();while(1);};

    u32 rov_interrupt_from_thread;
    u32 rov_interrupt_to_thread;
    u32 rov_thread_switch_interrupt_flag;
    /* 异常钩子函数 */
    static u8(*rov_exception_hook)(void* context) = 0;

    /* 异常栈（用于保存线程切换时的寄存器） */
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
    } ROV_NO_OPTIMIZE;

    /* 线程栈 */
    struct thread_stack_frame
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
        struct exception_stack_frame exception_stack_frame;
    } ROV_NO_OPTIMIZE;

    /* 异常信息 */
    struct exception_info
    {
        u32 exc_return;
        struct thread_stack_frame stack_frame;
    } ROV_NO_OPTIMIZE;

    /**
     * @brief 初始化线程栈
     * @param  thread_entry     线程入口函数
     * @param  parameter        函数参数
     * @param  stack_addr       栈地址
     * @param  thread_exit      线程退出函数
     * @return u8*
     * @note 该函数用于构建线程上下文
     */
    u8* rov_thread_stack_init(void* thread_entry, void* parameter, u8* stack_addr, void* thread_exit)
    {
        struct thread_stack_frame* stack_frame; //栈结构指针
        u8* stack_pointer; //栈指针

        /* 创建栈结构并对齐 */
        stack_pointer = stack_addr + sizeof(u32);
        stack_pointer = (u8*)ROV_ALIGN_DOWN((u32)(stack_pointer), 8);
        stack_pointer -= sizeof(struct thread_stack_frame);
        stack_frame = (struct thread_stack_frame*)stack_pointer;

        /* 初始化寄存器为默认值 */
        for (unsigned long i = 0; i < sizeof(struct thread_stack_frame) / sizeof(u32); i++)
        {
            ((u32*)stack_frame)[i] = U32_MAX;
        }

        stack_frame->exception_stack_frame.r0 = (unsigned long)parameter;       /* r0保存函数参数 */
        stack_frame->exception_stack_frame.r1 = 0;                              /* r1 */
        stack_frame->exception_stack_frame.r2 = 0;                              /* r2 */
        stack_frame->exception_stack_frame.r3 = 0;                              /* r3 */
        stack_frame->exception_stack_frame.r12 = 0;                             /* r12 */
        stack_frame->exception_stack_frame.lr = (unsigned long)thread_exit;     /* lr保存线程退出函数 */
        stack_frame->exception_stack_frame.pc = (unsigned long)thread_entry;    /* pc保存线程入口函数地址 */
        stack_frame->exception_stack_frame.psr = 0x01000000L;                   /* PSR */

#if USING_FPU
        stack_frame->flag = 0;
#endif

        /* 返回获得的栈地址 */
        return stack_frame;
    }

    /**
     * @brief 设置异常处理函数
     * @param  exception_handle 异常处理函数指针
     */
    void rov_exception_install(u8(*exception_handle)(void* context))
    {
        rov_exception_hook = exception_handle;
    }

    //DONE:全局中断控制
    /**
     * @brief 关全局中断
     * @return 当前中断状态
     */
    static ROV_ALWAYS_INLINE u32 rov_interrupt_disable(void)
    {
        /**
         * 读取PRIMASK寄存器的值到r0
         * 关闭全局中断
         * 函数返回
         */
#ifdef __GNUC__
        u32 now_level = 0;
        __asm volatile
        ("MRS %0, PRIMASK\n""CPSID I\n" : "=r" (now_level) :: "memory");
        return now_level;
#endif
#ifdef __CC_ARM
        __asm
        {
            MRS     r0, PRIMASK
            CPSID   I
        }
#endif
        /* 此时r0中存储了PRIMASK的值 */
    }

    /**
     * @brief 开全局中断
     * @param  basepri          要恢复的中断状态
     */
    static ROV_ALWAYS_INLINE void rov_interrupt_enable(u32 basepri)
    {
        /**
         * 将basepri的值写入PRIMASK寄存器
         * 函数返回
         */
#ifdef __GNUC__
        __asm volatile
        ("MSR   PRIMASK, %0" :: "r" (basepri) : "memory");
#endif
#ifdef __CC_ARM
        __asm
        {
            msr PRIMASK, basepri
        }
#endif
    }

    /**
     * @brief 进入死循环
     * @return ROV_WEAK
     */
    ROV_WEAK void rov_cpu_shutdown(void)
    {
        ROV_ASSERT(0);
    }

    /**
     * @brief 软件复位
     * @return ROV_WEAK
     */
    ROV_WEAK void rov_cpu_reset(void)
    {
        SCB_AIRCR = SCB_RESET_VALUE;
    }







    //TODO:systick函数编写
    //XXX:从FreeRTOS照抄的systick函数
    ROV_WEAK void SysTickInterrupt_Initer(void)
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

    ROV_WEAK void SysTickInterrupt_Handler(void)
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

    //XXX:从rtt抄的systick函数
    void SysTickInterrupt_Handler(void)
    {
        struct rt_thread* thread;
        rt_base_t level;

        level = rt_hw_interrupt_disable();

        /* increase the global tick */
        ++rt_tick;

        /* check time slice */
        thread = rt_thread_self();

        --thread->remaining_tick;
        if (thread->remaining_tick == 0)
        {
            /* change to initialized tick */
            thread->remaining_tick = thread->init_tick;
            thread->stat |= RT_THREAD_STAT_YIELD;

            rt_hw_interrupt_enable(level);
            rt_schedule();
        }
        else
        {
            rt_hw_interrupt_enable(level);
        }

        /* check timer */
        rt_timer_check();
    }





#ifdef __cplusplus
}
#endif

#endif