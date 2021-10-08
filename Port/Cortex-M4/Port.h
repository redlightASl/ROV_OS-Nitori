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

#if /* ARMCC */ (  (defined ( __CC_ARM ) && defined ( __TARGET_FPU_VFP ))    \
    /* Clang */ || (defined ( __CLANG_ARM ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)) \
    /* IAR */   || (defined ( __ICCARM__ ) && defined ( __ARMVFP__ ))        \
    /* GNU */   || (defined ( __GNUC__ ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)) )
#define USE_FPU   1
#else
#define USE_FPU   0
#endif



#define NVIC_INT_CTRL       0xE000ED04      // 中断控制及状态寄存器
#define NVIC_PENDSVSET      0x10000000      // 触发软件中断的值
#define NVIC_SYSPRI2        0xE000ED22      // 系统优先级寄存器
#define NVIC_PENDSV_PRI     0x000000FF      // 配置优先级

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


























//XXX：从liteOS抄的任务栈初始化函数
LITE_OS_SEC_TEXT_MINOR VOID OsTaskExit(VOID)
{
    __disable_irq();
    while (1) { }
}

LITE_OS_SEC_TEXT_INIT VOID *OsTaskStackInit(UINT32 taskId, UINT32 stackSize, VOID *topStack)
{
    TaskContext *taskContext = NULL;

    OsStackInit(topStack, stackSize);
    taskContext = (TaskContext *)(((UINTPTR)topStack + stackSize) - sizeof(TaskContext));

#ifdef LOSCFG_ARCH_FPU_ENABLE
    taskContext->excReturn = 0xFFFFFFFD;
#endif

    taskContext->R4  = 0x04040404L;
    taskContext->R5  = 0x05050505L;
    taskContext->R6  = 0x06060606L;
    taskContext->R7  = 0x07070707L;
    taskContext->R8  = 0x08080808L;
    taskContext->R9  = 0x09090909L;
    taskContext->R10 = 0x10101010L;
    taskContext->R11 = 0x11111111L;
    taskContext->PriMask = 0;
    taskContext->R0  = taskId;
    taskContext->R1  = 0x01010101L;
    taskContext->R2  = 0x02020202L;
    taskContext->R3  = 0x03030303L;
    taskContext->R12 = 0x12121212L;
    taskContext->LR  = (UINT32)OsTaskExit;
    taskContext->PC  = (UINT32)OsTaskEntry;
    taskContext->xPSR = 0x01000000L;

    return (VOID *)taskContext;
}























/* 异常服务函数表 */










/* 全局中断控制 */
//TODO:这里预计使用内嵌汇编而不是独立的.S文件
//XXX：freertos中断管理
#define portENTER_CRITICAL()                      vPortEnterCritical()
#define portEXIT_CRITICAL()                       vPortExitCritical()
void vPortEnterCritical(void)
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;

    /* This is not the interrupt safe version of the enter critical function so
     * assert() if it is being called from an interrupt context.  Only API
     * functions that end in "FromISR" can be used in an interrupt.  Only assert if
     * the critical nesting count is 1 to protect against recursive calls if the
     * assert function also uses a critical section. */
    if (uxCriticalNesting == 1)
    {
        configASSERT((portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK) == 0);
    }
}

void vPortExitCritical(void)
{
    configASSERT(uxCriticalNesting);
    uxCriticalNesting--;

    if (uxCriticalNesting == 0)
    {
        portENABLE_INTERRUPTS();
    }
}

#define portDISABLE_INTERRUPTS()                  vPortRaiseBASEPRI()
#define portENABLE_INTERRUPTS()                   vPortSetBASEPRI( 0 )
static portFORCE_INLINE void vPortRaiseBASEPRI(void)
{
    uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

    __asm
    {
        /* Set BASEPRI to the max syscall priority to effect a critical
         * section. */
         /* *INDENT-OFF* */
        msr basepri, ulNewBASEPRI
        dsb
        isb
        /* *INDENT-ON* */
    }
}

static portFORCE_INLINE void vPortSetBASEPRI(uint32_t ulBASEPRI)
{
    __asm
    {
        /* Barrier instructions are not used as this function is only used to
         * lower the BASEPRI value. */
         /* *INDENT-OFF* */
        msr basepri, ulBASEPRI
        /* *INDENT-ON* */
    }
}

//XXX：ucos2中断管理
#define  OS_ENTER_CRITICAL()  {cpu_sr = OS_CPU_SR_Save();}
#define  OS_EXIT_CRITICAL()   {OS_CPU_SR_Restore(cpu_sr);}

/*
 * #define  OS_ENTER_CRITICAL()  {cpu_sr = OS_CPU_SR_Save();}
 * 状态寄存器入栈
 */
OS_CPU_SR_Save:
    MRS     R0, CPSR
    ORR     R1, R0, #OS_CPU_ARM_CONTROL_INT_DIS                 @ Set IRQ and FIQ bits in CPSR to disable all interrupts.
    MSR     CPSR_c, R1
    BX      LR                                                  @ Disabled, return the original CPSR contents in R0.

/*
 * #define  OS_EXIT_CRITICAL()   {OS_CPU_SR_Restore(cpu_sr);}
 * 状态寄存器出栈
 */
OS_CPU_SR_Restore:
    MSR     CPSR_c, R0
    BX      LR

















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
    }
}































/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
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



//XXX:从FreeRTOS照抄的systick函数
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

//XXX:从FreeRTOS照抄的PendSV中断服务函数
void vPortSVCHandler(void);
__asm void vPortSVCHandler(void)
{
    /* *INDENT-OFF* */
    PRESERVE8

        /* Get the location of the current TCB. */
        ldr r3, = pxCurrentTCB
        ldr r1, [r3]
        ldr r0, [r1]
        /* Pop the core registers. */
        ldmia r0 !, { r4 - r11,r14 }
        msr psp, r0
        isb
        mov r0, # 0
        msr basepri, r0
        bx r14
        /* *INDENT-ON* */
}





































#endif