# 这里是面向Cortex-M4的汇编移植文件
.cpu cortex-m4
.syntax unified
.thumb
.text
.align 2

.equ    SCB_VTOR,           0xE000ED08              /* 向量表偏移寄存器 */
.equ    NVIC_INT_CTRL,      0xE000ED04              /* 中断控制状态寄存器 */
.equ    NVIC_SYSPRI2,       0xE000ED20              /* 系统优先级寄存器 (2) */
.equ    NVIC_SYSPRI14,      0xE000ED22              /* 系统优先级寄存器 (14) */
.equ    NVIC_PENDSV_PRI,    0x00FF0000              /* PendSV中断优先级 (设置为最低) */
.equ    NVIC_PENDSVSET,     0x10000000              /* PendSV中断触发复位值 */

    .extern  OSRunning
    .extern  OSPrioCur
    .extern  OSPrioHighRdy
    .extern  OSTCBCur
    .extern  OSTCBHighRdy
    .extern  OSIntExit
    .extern  OSTaskSwHook
    .extern  OS_CPU_ExceptStkBase
    .extern  OS_KA_BASEPRI_Boundary


    .global  OSStartHighRdy
    .global  OS_CPU_SR_Save
    .global  OS_CPU_SR_Restore
    .global  OSCtxSw
    .global  OSIntCtxSw
    .global  OS_CPU_PendSVHandler

#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
    .global  OS_CPU_FP_Reg_Push
    .global  OS_CPU_FP_Reg_Pop
#endif

/**
 * rov_float_register_pop()
 * FPU寄存器出栈
 */
#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
.global rov_float_register_pop
.type rov_float_register_pop, %function
rov_float_register_pop:
    VLDMIA  r0!, {S16-S31} /* 把寄存器值存储到新线程栈 */
    LDR     r1, =OSTCBHighRdy //更新OSTCBHighRdy->OSTCBStkPtr到新的线程栈
    LDR     r2, [r1]
    STR     r0, [r2]
    BX      LR
#endif


/**
 * rov_float_register_push()
 * FPU寄存器压栈
 */
#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
.global rov_float_register_push
.type rov_float_register_push, %function
rov_float_register_push:
    MRS     r1, PSP //PSP即线程SP
    CBZ     r1, _FLOAT_REGISTER_NO_SAVE //第一次跳过FPU寄存器保存

    VSTMDB  r0!, {S16-S31} /* FPU寄存器压栈 */
    LDR     r1, =OSTCBCur //更新OSTCBCur->OSTCBStkPtr
    LDR     r2, [r1]
    STR     r0, [r2]
_FLOAT_REGISTER_NO_SAVE:
    BX      LR
#endif


//TODO:优化线程切换函数
/*
 * rov_context_switch(from, to);
 * rov_context_switch_interrupt(from, to);
 * r0 --> from
 * r1 --> to
 * 线程上下文切换底层实现
 */
.global rov_context_switch_interrupt
.type rov_context_switch_interrupt, %function
.global rov_context_switch
.type rov_context_switch, %function

rov_context_switch_interrupt:
rov_context_switch:
    /* rov_thread_switch_interrupt_flag标志置1 */
    LDR     r2, =rov_thread_switch_interrupt_flag
    LDR     r3, [r2]
    CMP     r3, #1
    BEQ     _reswitch
    MOV     r3, #1
    STR     r3, [r2]

    LDR     r2, =rov_interrupt_from_thread   /* 设置rov_interrupt_from_thread */
    STR     r0, [r2]

_reswitch:
    LDR     r2, =rov_interrupt_to_thread     /* 设置rov_interrupt_to_thread */
    STR     r1, [r2]

    LDR r0, =NVIC_INT_CTRL /* 触发PendSV中断 */
    LDR r1, =NVIC_PENDSVSET
    STR r1, [r0]
    BX  LR

/*
 * rov_context_switch_to(to);
 * r0 --> to
 * 无源头切换线程
 */
.global rov_context_switch_to
.type rov_context_switch_to, %function
rov_context_switch_to:
    LDR r1, =rov_interrupt_to_thread
    STR r0, [r1]

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    MRS     r2, CONTROL         /* read */
    BIC     r2, #0x04           /* modify */
    MSR     CONTROL, r2         /* write-back */
#endif
    LDR r1, =rov_interrupt_from_thread
    MOV r0, #0x0
    STR r0, [r1]

    LDR     r1, =rov_thread_switch_interrupt_flag
    MOV     r0, #1
    STR     r0, [r1]

    /* 设置PendSV中断优先级 */
    LDR r0, =NVIC_SYSPRI2
    LDR r1, =NVIC_PENDSV_PRI
    LDR.W   r2, [r0,#0x00]
    ORR     r1,r1,r2 
    STR     r1, [r0]

    LDR r0, =NVIC_INT_CTRL      /* 触发PendSV中断 */
    LDR r1, =NVIC_PENDSVSET
    STR r1, [r0]

    /* 存储MSP */
    LDR     r0, =SCB_VTOR
    LDR     r0, [r0]
    LDR     r0, [r0]
    NOP
    MSR     msp, r0

    /* 在处理器级别使能中断 */
    CPSIE   F
    CPSIE   I

    /* never reach here! */











//TODO:pendsv中断处理函数优化
//ucosii的pendsv中断服务函数
@********************************************************************************************************
@                                       HANDLE PendSV EXCEPTION
@                                   void OS_CPU_PendSVHandler(void)
@
@ Note(s) : 1) PendSV is used to cause a context switch.  This is a recommended method for performing
@              context switches with Cortex-M.  This is because the Cortex-M auto-saves half of the
@              processor context on any exception, and restores same on return from exception.  So only
@              saving of R4-R11 & R14 is required and fixing up the stack pointers. Using the PendSV exception
@              this way means that context saving and restoring is identical whether it is initiated from
@              a thread or occurs due to an interrupt or exception.
@
@           2) Pseudo-code is:
@              a) Get the process SP
@              b) Save remaining regs r4-r11 & r14 on process stack;
@              c) Save the process SP in its TCB, OSTCBCur->OSTCBStkPtr = SP;
@              d) Call OSTaskSwHook();
@              e) Get current high priority, OSPrioCur = OSPrioHighRdy;
@              f) Get current ready thread TCB, OSTCBCur = OSTCBHighRdy;
@              g) Get new process SP from TCB, SP = OSTCBHighRdy->OSTCBStkPtr;
@              h) Restore R4-R11 and R14 from new process stack;
@              i) Perform exception return which will restore remaining context.
@
@           3) On entry into PendSV handler:
@              a) The following have been saved on the process stack (by processor):
@                 xPSR, PC, LR, R12, R0-R3
@              b) Processor mode is switched to Handler mode (from Thread mode)
@              c) Stack is Main stack (switched from Process stack)
@              d) OSTCBCur      points to the OS_TCB of the task to suspend
@                 OSTCBHighRdy  points to the OS_TCB of the task to resume
@
@           4) Since PendSV is set to lowest priority in the system (by OSStartHighRdy() above), we
@              know that it will only be run when no other exception or interrupt is active, and
@              therefore safe to assume that context being switched out was using the process stack (PSP).
@
@           5) Increasing priority using a write to BASEPRI does not take effect immediately.
@              (a) IMPLICATION  This erratum means that the instruction after an MSR to boost BASEPRI
@                  might incorrectly be preempted by an insufficient high priority exception.
@
@              (b) WORKAROUND  The MSR to boost BASEPRI can be replaced by the following code sequence:
@
@                  CPSID i
@                  MSR to BASEPRI
@                  DSB
@                  ISB
@                  CPSIE i
@********************************************************************************************************
.thumb_func
OS_CPU_PendSVHandler:
    CPSID   I                                                   @ Cortex-M7 errata notice. See Note #5
    MOVW    R2, #:lower16:OS_KA_BASEPRI_Boundary                @ Set BASEPRI priority level required for exception preemption
    MOVT    R2, #:upper16:OS_KA_BASEPRI_Boundary
    LDR     R1, [R2]
    MSR     BASEPRI, R1
    DSB
    ISB
    CPSIE   I

    MRS     R0, PSP                                             @ PSP is process stack pointer
    STMFD   R0!, {R4-R11, R14}                                  @ Save remaining regs r4-11, R14 on process stack

    MOVW    R5, #:lower16:OSTCBCur                              @ OSTCBCur->OSTCBStkPtr = SP;
    MOVT    R5, #:upper16:OSTCBCur
    LDR     R1, [R5]
    STR     R0, [R1]                                            @ R0 is SP of process being switched out

                                                                @ At this point, entire context of process has been saved
    MOV     R4, LR                                              @ Save LR exc_return value
    BL      OSTaskSwHook                                        @ Call OSTaskSwHook() for FPU Push & Pop

    MOVW    R0, #:lower16:OSPrioCur                             @ OSPrioCur   = OSPrioHighRdy;
    MOVT    R0, #:upper16:OSPrioCur
    MOVW    R1, #:lower16:OSPrioHighRdy
    MOVT    R1, #:upper16:OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]

    MOVW    R1, #:lower16:OSTCBHighRdy                          @ OSTCBCur  = OSTCBHighRdy;
    MOVT    R1, #:upper16:OSTCBHighRdy
    LDR     R2, [R1]
    STR     R2, [R5]

    ORR     LR,  R4, #0x04                                      @ Ensure exception return uses process stack
    LDR     R0,  [R2]                                           @ R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;
    LDMFD   R0!, {R4-R11, R14}                                  @ Restore r4-11, R14 from new process stack
    MSR     PSP, R0                                             @ Load PSP with new process SP

    MOV     R2, #0                                              @ Restore BASEPRI priority level to 0
    MSR     BASEPRI, R2
    BX      LR                                                  @ Exception return will restore remaining context

.end




//来自atomthreads的pendsv中断处理函数
.global pend_sv_handler
.func   pend_sv_handler
.type   pend_sv_handler,%function
.thumb_func
pend_sv_handler:
    /**
     * Disable interrupts. No need to check if they were enabled because,
     * well, we're an interrupt handler. Duh...
     */
    movs    r0,         #1
    msr     PRIMASK,    r0
    
    /**
     * Clear PendSv pending bit. There seems to exist a hardware race condition
     * in the NVIC that can prevent automatic clearing of the PENDSVSET. See
     * http://embeddedgurus.com/state-space/2011/09/whats-the-state-of-your-cortex/
     */
     ldr    r0,         = SCB_ICSR
     ldr    r1,         = PENDSVCLR
     str    r1,         [r0, #0]

    /**
     * Check if running and next thread are really different.
     * From here on we have
     * r0 = &ctx_switch_info
     * r1 = ctx_switch_info.running_tcb
     * r2 = ctx_switch_info.next_tcb
     *
     * If r1 == r2 we can skip the context switch. This may theoretically
     * happen if the running thread gets scheduled out and in again by
     * multiple nested or tail-chained ISRs before the PendSv handler
     * gets called.
     */
    ldr     r0,         = CTX_SW_NFO
    ldr     r1,         [r0, #CTX_RUN_OFF]
    ldr     r2,         [r0, #CTX_NEXT_OFF]
    cmp     r1,         r2
    beq     no_switch

    /**
     * Copy running thread's process stack pointer to r3 and use it to push
     * the thread's register context on its stack
     */
    mrs     r3,         PSP

#if defined(THUMB_2)
    /**
     * Save old thread's context on Cortex-M[34]
     */

#if defined(WITH_FPU)
    /* Check if FPU was used by thread and store registers if necessary */
    tst     lr,         FPU_USED
    it      eq
    vstmdbeq  r3!,      {s16-s31}

    /**
     * TODO: Defer stacking FPU context by disabling FPU and using a
     * fault handler to store the FPU registers if another thread
     * tries using it
     */
#endif // WITH_FPU
    
    /* Push running thread's remaining registers on stack */
    stmdb   r3!,        {r4-r11, lr}

#else // !THUMB2

    /**
     * Save old thread's register context on Cortex-M0.
     * Push running thread's remaining registers on stack.
     * Thumb-1 can use stm only on low registers, so we
     * have to do this in two steps.
     */

    /* Reserve space for r8-r11 + exc_return before storing r4-r7 */
    subs    r3,         r3,     #36
    stmia   r3!,        {r4-r7}

    /**
     * Move r8-r11 to low registers and use store multiple with automatic
     * post-increment to push them on the stack
     */
    mov     r4,         r8
    mov     r5,         r9
    mov     r6,         r10
    mov     r7,         r11
    stmia   r3!,        {r4-r7}

    /**
     * Move lr (contains the exc_return code) to low registers and store it
     * on the stack.
     */
    mov     r4,         lr
    str     r4,         [r3, #0]

    /* Re-adjust r3 to point at top of stack */
    subs    r3,         r3, #32
#endif // !THUMB_2
    /**
     * Address of running TCB still in r1. Store thread's current stack top
     * into its sp_save_ptr, which is the struct's first element.
     */
    str     r3,         [r1, #0]
    
    /**
     * ctx_switch_info.next_tcb is going to become ctx_switch_info.running_tcb,
     * so we update the pointer.
     */
    str     r2,         [r0, #CTX_RUN_OFF]
    
#if defined(__NEWLIB__)
    /**
     * Store the thread's reentry context address in _impure_ptr. This
     * will have been stored in ctx_switch_info.reent.
     */ 
    ldr     r4,         [r0, #CTX_REENT_OFF]
    ldr     r3,         = _impure_ptr
    str     r4,         [r3, #0]
#endif

    /**
     * Fetch next thread's stack pointer from its TCB's sp_save_ptr and restore
     * the thread's register context.
     */
    ldr     r3,         [r2, #0]

#if defined(THUMB_2)

    /* Cortex-M[34], restore thread's task stack frame */
    ldmia   r3!,        {r4-r11, lr}

#if defined(WITH_FPU)
    /**
     * Check if FPU was used by new thread and restore registers if necessary.
     */
    tst     lr,         FPU_USED
    it      eq
    vldmiaeq  r3!,      {s16-s31}

    /**
     * TODO: only restore FPU registers if FPU was used by another thread
     * between this thread being scheduled out and now.
     */
#endif // WITH_FPU
#else // !THUMB_2

    /**
     * Thread restore for Cortex-M0
     * Restore thread's task stack frame. Because thumb 1 only supports
     * load multiple on low register, we have to do it in two steps and
     * adjust the stack pointer manually.
     */

    /* Restore high registers */
    adds    r3,         r3, #16
    ldmia   r3!,        {r4-r7}
    mov     r8,         r4
    mov     r9,         r5
    mov     r10,        r6
    mov     r11,        r7

    /* Restore lr */
    ldr     r4,         [r3, #0]
    mov     lr,         r4
    subs    r3,         r3, #32

    /**
     * Restore r4-r7 and adjust r3 to point at the top of the exception
     * stack frame.
     */
    ldmia   r3!,        {r4-r7}
    adds    r3,         r3, #20
#endif // !THUMB_2

    /* Set process stack pointer to new thread's stack*/
    msr     PSP,        r3

no_switch:
    /* Re-enable interrupts */
    movs    r0,         #0
    msr     PRIMASK,    r0
    
    /* Return to new thread */
    bx      lr
    nop
.size   pend_sv_handler, . - pend_sv_handler
.endfunc


//来自rtt的pendsv中断处理函数
/* r0 --> switch from thread stack
 * r1 --> switch to thread stack
 * psr, pc, lr, r12, r3, r2, r1, r0 are pushed into [from] stack
 */
.global PendSV_Handler
.type PendSV_Handler, %function
PendSV_Handler:
    /* disable interrupt to protect context switch */
    MRS r2, PRIMASK
    CPSID   I

    /* get rt_thread_switch_interrupt_flag */
    LDR r0, =rt_thread_switch_interrupt_flag
    LDR r1, [r0]
    CBZ r1, pendsv_exit         /* pendsv already handled */

    /* clear rt_thread_switch_interrupt_flag to 0 */
    MOV r1, #0x00
    STR r1, [r0]

    LDR r0, =rt_interrupt_from_thread
    LDR r1, [r0]
    CBZ r1, switch_to_thread    /* skip register save at the first time */

    MRS r1, psp                 /* get from thread stack pointer */
    
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    TST     lr, #0x10           /* if(!EXC_RETURN[4]) */
    VSTMDBEQ r1!, {d8 - d15}    /* push FPU register s16~s31 */
#endif
    
    STMFD   r1!, {r4 - r11}     /* push r4 - r11 register */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    MOV     r4, #0x00           /* flag = 0 */

    TST     lr, #0x10           /* if(!EXC_RETURN[4]) */
    MOVEQ   r4, #0x01           /* flag = 1 */

    STMFD   r1!, {r4}           /* push flag */
#endif

    LDR r0, [r0]
    STR r1, [r0]                /* update from thread stack pointer */

switch_to_thread:
    LDR r1, =rt_interrupt_to_thread
    LDR r1, [r1]
    LDR r1, [r1]                /* load thread stack pointer */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    LDMFD   r1!, {r3}           /* pop flag */
#endif

    LDMFD   r1!, {r4 - r11}     /* pop r4 - r11 register */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    CMP     r3,  #0             /* if(flag_r3 != 0) */
    VLDMIANE  r1!, {d8 - d15}   /* pop FPU register s16~s31 */
#endif

    MSR psp, r1                 /* update stack pointer */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    ORR     lr, lr, #0x10       /* lr |=  (1 << 4), clean FPCA. */
    CMP     r3,  #0             /* if(flag_r3 != 0) */
    BICNE   lr, lr, #0x10       /* lr &= ~(1 << 4), set FPCA. */
#endif

pendsv_exit:
    /* restore interrupt */
    MSR PRIMASK, r2

    ORR lr, lr, #0x04
    BX  lr