#这里是面向Cortex-M4的汇编移植文件 
.cpu cortex-m4
.syntax unified
.thumb
.text

.equ    SCB_VTOR,           0xE000ED08              /* 向量表偏移寄存器 */
.equ    NVIC_INT_CTRL,      0xE000ED04              /* 中断控制状态寄存器 */
.equ    NVIC_SYSPRI2,       0xE000ED20              /* 系统优先级寄存器 (2) */
.equ    NVIC_PENDSV_PRI,    0x00FF0000              /* PendSV中断优先级 (设置为最低) */
.equ    NVIC_PENDSVSET,     0x10000000              /* PendSV中断触发复位值 */

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

/* XXX:直接照抄RT-Thread的PendSV中断服务函数
 * r0 --> switch from thread stack
 * r1 --> switch to thread stack
 * psr, pc, lr, r12, r3, r2, r1, r0 are pushed into [from] stack
 */
.global PendSV_Handler
.type PendSV_Handler, %function
PendSV_Handler:
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
    VSTMDBEQ r1!, {d8 - d15}
#endif
    
    STMFD   r1!, {r4 - r11}

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    MOV     r4, #0x00

    TST     lr, #0x10
    MOVEQ   r4, #0x01

    STMFD   r1!, {r4} 
#endif

    LDR r0, [r0]
    STR r1, [r0]

switch_to_thread:
    LDR r1, =rov_interrupt_to_thread
    LDR r1, [r1]
    LDR r1, [r1] 

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    LDMFD   r1!, {r3} 
#endif

    LDMFD   r1!, {r4 - r11}

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    CMP     r3,  #0
    VLDMIANE  r1!, {d8 - d15}
#endif

    MSR psp, r1

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    ORR     lr, lr, #0x10 
    CMP     r3,  #0   
    BICNE   lr, lr, #0x10   
#endif

pendsv_exit:
    MSR PRIMASK, r2

    ORR lr, lr, #0x04
    BX  lr

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
