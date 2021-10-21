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

//来自ucosii的汇编
    .extern  OSRunning                                          @ External references
    .extern  OSPrioCur
    .extern  OSPrioHighRdy
    .extern  OSTCBCur
    .extern  OSTCBHighRdy
    .extern  OSIntExit
    .extern  OSTaskSwHook
    .extern  OS_CPU_ExceptStkBase
    .extern  OS_KA_BASEPRI_Boundary


    .global  OSStartHighRdy                                     @ Functions declared in this file
    .global  OS_CPU_SR_Save
    .global  OS_CPU_SR_Restore
    .global  OSCtxSw
    .global  OSIntCtxSw
    .global  OS_CPU_PendSVHandler

#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
    .global  OS_CPU_FP_Reg_Push
    .global  OS_CPU_FP_Reg_Pop
#endif



//来自ucosii的汇编

/**
 * rov_float_register_pop()
 * FPU寄存器出栈
 */
#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
.global rov_float_register_pop
.type rov_float_register_pop, %function
rov_float_register_pop:
    VLDMIA  r0!, {S16-S31} /* 把寄存器值存储到新线程栈 */
    LDR     r1, =OSTCBHighRdy //TODO：更新OSTCBHighRdy->OSTCBStkPtr到新的线程栈
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
    CBZ     r1, OS_CPU_FP_nosave //第一次跳过FPU寄存器保存

    VSTMDB  r0!, {S16-S31} /* FPU寄存器压栈 */
    LDR     r1, =OSTCBCur //TODO:更新OSTCBCur->OSTCBStkPtr
    LDR     r2, [r1]
    STR     r0, [r2]
OS_CPU_FP_nosave:
    BX      LR
#endif


/*
 * rov_interrupt_disable();
 * 关全局中断底层实现
 */
.global rov_interrupt_disable
.type rov_interrupt_disable, %function
rov_interrupt_disable:
    MRS     r0, PRIMASK
    CPSID   I
    BX      LR

/*
 * rov_interrupt_enable();
 * 开全局中断底层实现
 */
.global rov_interrupt_enable
.type rov_interrupt_enable, %function
rov_interrupt_enable:
    MSR     PRIMASK, r0
    BX      LR



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










//ucosii的线程调度启动函数

@********************************************************************************************************
@                                         START MULTITASKING
@                                      void OSStartHighRdy(void)
@
@ Note(s) : 1) This function triggers a PendSV exception (essentially, causes a context switch) to cause
@              the first task to start.
@
@           2) During task execution, PSP is used as the stack pointer.
@              When an exception occurs, the core will switch to MSP until the exception return.
@
@           3) OSStartHighRdy() MUST:
@              a) Setup PendSV exception priority to lowest;
@              b) Set initial PSP to 0, to tell context switcher this is first run;
@              c) Set the main stack to OS_CPU_ExceptStkBase
@              d) Set OSRunning to TRUE;
@              e) Get current high priority, OSPrioCur = OSPrioHighRdy;
@              f) Get current ready thread TCB, OSTCBCur = OSTCBHighRdy;
@              g) Get new process SP from TCB, SP = OSTCBHighRdy->OSTCBStkPtr;
@              h) Restore R0-R11 and R14 from new process stack;
@              i) Enable interrupts (tasks will run with interrupts enabled).
@********************************************************************************************************

.thumb_func
OSStartHighRdy:
    CPSID   I                                                   @ Prevent interruption during context switch
    MOVW    R0, #:lower16:NVIC_SYSPRI14                         @ Set the PendSV exception priority
    MOVT    R0, #:upper16:NVIC_SYSPRI14

    MOVW    R1, #:lower16:NVIC_PENDSV_PRI
    MOVT    R1, #:upper16:NVIC_PENDSV_PRI
    STRB    R1, [R0]

    MOVS    R0, #0                                              @ Set the PSP to 0 for initial context switch call
    MSR     PSP, R0

    MOVW    R0, #:lower16:OS_CPU_ExceptStkBase                  @ Initialize the MSP to the OS_CPU_ExceptStkBase
    MOVT    R0, #:upper16:OS_CPU_ExceptStkBase
    LDR     R1, [R0]
    MSR     MSP, R1

    BL      OSTaskSwHook                                        @ Call OSTaskSwHook() for FPU Push & Pop

    LDR     R0, =OSRunning                                      @ OSRunning = TRUE
    MOVS    R1, #1
    STRB    R1, [R0]

    MOVW    R0, #:lower16:OSPrioCur                             @ OSPrioCur   = OSPrioHighRdy;
    MOVT    R0, #:upper16:OSPrioCur
    MOVW    R1, #:lower16:OSPrioHighRdy
    MOVT    R1, #:upper16:OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]

    MOVW    R0, #:lower16:OSTCBCur                              @ OSTCBCur  = OSTCBHighRdy;
    MOVT    R0, #:upper16:OSTCBCur
    MOVW    R1, #:lower16:OSTCBHighRdy
    MOVT    R1, #:upper16:OSTCBHighRdy
    LDR     R2, [R1]
    STR     R2, [R0]

    LDR     R0, [R2]                                            @ R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;
    MSR     PSP, R0                                             @ Load PSP with new process SP

    MRS     R0, CONTROL
    ORR     R0, R0, #2
    MSR     CONTROL, R0
    ISB                                                         @ Sync instruction stream

    LDMFD    SP!, {R4-R11, LR}                                  @ Restore r4-11, lr from new process stack
    LDMFD    SP!, {R0-R3}                                       @ Restore r0, r3
    LDMFD    SP!, {R12, LR}                                     @ Load R12 and LR
    LDMFD    SP!, {R1, R2}                                      @ Load PC and discard xPSR
    CPSIE    I
    BX       R1






//ucosii的线程切换函数
@********************************************************************************************************
@                       PERFORM A CONTEXT SWITCH (From task level) - OSCtxSw()
@                   PERFORM A CONTEXT SWITCH (From interrupt level) - OSIntCtxSw()
@
@ Note(s) : 1) OSCtxSw() is called when OS wants to perform a task context switch.  This function
@              triggers the PendSV exception which is where the real work is done.
@
@           2) OSIntCtxSw() is called by OSIntExit() when it determines a context switch is needed as
@              the result of an interrupt.  This function simply triggers a PendSV exception which will
@              be handled when there are no more interrupts active and interrupts are enabled.
@********************************************************************************************************

.thumb_func
OSCtxSw:
OSIntCtxSw:
    LDR     R0, =NVIC_INT_CTRL                                  @ Trigger the PendSV exception (causes context switch)
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR




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




    