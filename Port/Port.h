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

 /*
 主要分成两个部分
 cpu移植函数
 硬件移植函数

 cpu移植函数分成两个子部分
     pendsv相关（线程切换与线程管理）
         采用独立的.S汇编文件实现
         因为涉及到RV32指令集的移植，使用独立的汇编文件方便修改
     中断管理相关
         采用c内嵌汇编实现
         因为内容不多且与内核应用联系紧密

 硬件移植函数又分为
     通用寄存器移植函数
         直接在Port.h中完成移植即可
         将通用寄存器以结构体的形式映射到内存，方便汇编-c融合操作
     DSA寄存器函数
         专用于硬件加速的外设寄存器与相关应用程序需要在HardwareAccelerate.c、.h文件中
         单独实现，作为移植的一部分
     HAL函数
         面向stm32-HAL库的函数接口，需要在Defines.h中以宏定义形式实现
     特殊硬件控制函数
         单独封装在Port.h中供上层程序调用
 */
 /* 异常服务函数表 */
 //TODO:这里预计使用内嵌汇编而不是独立的.S文件


    /* 临界区管理函数 */
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
extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);

#define portDISABLE_INTERRUPTS()                  vPortRaiseBASEPRI()
#define portENABLE_INTERRUPTS()                   vPortSetBASEPRI( 0 )
#define portENTER_CRITICAL()                      vPortEnterCritical()
#define portEXIT_CRITICAL()                       vPortExitCritical()
#define portSET_INTERRUPT_MASK_FROM_ISR()         ulPortRaiseBASEPRI()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    vPortSetBASEPRI( x )
















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

void xPortSysTickHandler(void);
void xPortSysTickHandler(void)
{
    /* The SysTick runs at the lowest interrupt priority, so when this interrupt
     * executes all interrupts must be unmasked.  There is therefore no need to
     * save and then restore the interrupt mask value as its value is already
     * known - therefore the slightly faster vPortRaiseBASEPRI() function is used
     * in place of portSET_INTERRUPT_MASK_FROM_ISR(). */
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

BaseType_t xPortStartScheduler( void )
{
    /* configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.
     * See https://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
    configASSERT( configMAX_SYSCALL_INTERRUPT_PRIORITY );

    /* This port can be used on all revisions of the Cortex-M7 core other than
     * the r0p1 parts.  r0p1 parts should use the port from the
     * /source/portable/GCC/ARM_CM7/r0p1 directory. */
    configASSERT( portCPUID != portCORTEX_M7_r0p1_ID );
    configASSERT( portCPUID != portCORTEX_M7_r0p0_ID );

    #if ( configASSERT_DEFINED == 1 )
        {
            volatile uint32_t ulOriginalPriority;
            volatile uint8_t * const pucFirstUserPriorityRegister = ( uint8_t * ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
            volatile uint8_t ucMaxPriorityValue;

            /* Determine the maximum priority from which ISR safe FreeRTOS API
             * functions can be called.  ISR safe functions are those that end in
             * "FromISR".  FreeRTOS maintains separate thread and ISR API functions to
             * ensure interrupt entry is as fast and simple as possible.
             *
             * Save the interrupt priority value that is about to be clobbered. */
            ulOriginalPriority = *pucFirstUserPriorityRegister;

            /* Determine the number of priority bits available.  First write to all
             * possible bits. */
            *pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

            /* Read the value back to see how many bits stuck. */
            ucMaxPriorityValue = *pucFirstUserPriorityRegister;

            /* The kernel interrupt priority should be set to the lowest
             * priority. */
            configASSERT( ucMaxPriorityValue == ( configKERNEL_INTERRUPT_PRIORITY & ucMaxPriorityValue ) );

            /* Use the same mask on the maximum system call priority. */
            ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

            /* Calculate the maximum acceptable priority group value for the number
             * of bits read back. */
            ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;

            while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
            {
                ulMaxPRIGROUPValue--;
                ucMaxPriorityValue <<= ( uint8_t ) 0x01;
            }

            #ifdef __NVIC_PRIO_BITS
                {
                    /* Check the CMSIS configuration that defines the number of
                     * priority bits matches the number of priority bits actually queried
                     * from the hardware. */
                    configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == __NVIC_PRIO_BITS );
                }
            #endif

            #ifdef configPRIO_BITS
                {
                    /* Check the FreeRTOS configuration that defines the number of
                     * priority bits matches the number of priority bits actually queried
                     * from the hardware. */
                    configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == configPRIO_BITS );
                }
            #endif

            /* Shift the priority group value back to its position within the AIRCR
             * register. */
            ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
            ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;

            /* Restore the clobbered interrupt priority register to its original
             * value. */
            *pucFirstUserPriorityRegister = ulOriginalPriority;
        }
    #endif /* conifgASSERT_DEFINED */

    /* Make PendSV and SysTick the lowest priority interrupts. */
    portNVIC_SHPR3_REG |= portNVIC_PENDSV_PRI;
    portNVIC_SHPR3_REG |= portNVIC_SYSTICK_PRI;

    /* Start the timer that generates the tick ISR.  Interrupts are disabled
     * here already. */
    vPortSetupTimerInterrupt();

    /* Initialise the critical nesting count ready for the first task. */
    uxCriticalNesting = 0;

    /* Ensure the VFP is enabled - it should be anyway. */
    prvEnableVFP();

    /* Lazy save always. */
    *( portFPCCR ) |= portASPEN_AND_LSPEN_BITS;

    /* Start the first task. */
    prvStartFirstTask();

    /* Should not get here! */
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not implemented in ports where there is nothing to return to.
     * Artificially force an assert. */
    configASSERT( uxCriticalNesting == 1000UL );
}

__asm uint32_t vPortGetIPSR( void )
{
/* *INDENT-OFF* */
    PRESERVE8

    mrs r0, ipsr
    bx r14
/* *INDENT-ON* */
}











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