#include "Thread.h"



//XXX：从LiteOS抄的NVIC管理
STATIC UINT32 g_curIrqNum = 0;

LITE_OS_SEC_BSS HwiHandleInfo g_hwiForm[LOSCFG_PLATFORM_HWI_LIMIT] = { 0 };

LITE_OS_SEC_DATA_VEC HWI_PROC_FUNC g_hwiVec[LOSCFG_PLATFORM_HWI_LIMIT] = {
    (HWI_PROC_FUNC)0,             /* [0] Top of Stack */
    (HWI_PROC_FUNC)Reset_Handler, /* [1] reset */
    (HWI_PROC_FUNC)IrqEntryV7M,   /* [2] NMI Handler */
    (HWI_PROC_FUNC)IrqEntryV7M,   /* [3] Hard Fault Handler */
    (HWI_PROC_FUNC)IrqEntryV7M,   /* [4] MPU Fault Handler */
    (HWI_PROC_FUNC)IrqEntryV7M,   /* [5] Bus Fault Handler */
    (HWI_PROC_FUNC)IrqEntryV7M,   /* [6] Usage Fault Handler */
    (HWI_PROC_FUNC)0,             /* [7] Reserved */
    (HWI_PROC_FUNC)0,             /* [8] Reserved */
    (HWI_PROC_FUNC)0,             /* [9] Reserved */
    (HWI_PROC_FUNC)0,             /* [10] Reserved */
    (HWI_PROC_FUNC)IrqEntryV7M,   /* [11] SVCall Handler */
    (HWI_PROC_FUNC)IrqEntryV7M,   /* [12] Debug Monitor Handler */
    (HWI_PROC_FUNC)0,             /* [13] Reserved */
    (HWI_PROC_FUNC)osPendSV,      /* [14] PendSV Handler */
    (HWI_PROC_FUNC)IrqEntryV7M,   /* [15] SysTick Handler */
};


LITE_OS_SEC_TEXT_MINOR VOID IrqEntryV7M(VOID)
{
    UINT32 hwiIndex;

    hwiIndex = __get_IPSR();
    g_curIrqNum = hwiIndex;
    OsIntHandle(hwiIndex, &g_hwiForm[hwiIndex]);

    if (OsTaskProcSignal() != 0) {
        OsSchedPreempt();
    }
}

UINT32 HalIrqUnmask(UINT32 hwiNum)
{
    UINT32 intSave;

    if (!HWI_NUM_VALID(hwiNum)) {
        return LOS_ERRNO_HWI_NUM_INVALID;
    }

    hwiNum -= OS_SYS_VECTOR_CNT;
    intSave = LOS_IntLock();
    NVIC_EnableIRQ((IRQn_Type)hwiNum);
    LOS_IntRestore(intSave);
    return LOS_OK;
}

UINT32 HalIrqSetPriority(UINT32 hwiNum, UINT8 priority)
{
    UINT32 intSave;

    if (!HWI_NUM_VALID(hwiNum)) {
        return LOS_ERRNO_HWI_NUM_INVALID;
    }

    if (!HWI_PRI_VALID(priority)) {
        return OS_ERRNO_HWI_PRIO_INVALID;
    }

    hwiNum -= OS_SYS_VECTOR_CNT;
    intSave = LOS_IntLock();
    NVIC_SetPriority((IRQn_Type)hwiNum, priority);
    LOS_IntRestore(intSave);

    return LOS_OK;
}

UINT32 HalIrqMask(HWI_HANDLE_T hwiNum)
{
    UINT32 intSave;

    if (!HWI_NUM_VALID(hwiNum)) {
        return LOS_ERRNO_HWI_NUM_INVALID;
    }

    hwiNum -= OS_SYS_VECTOR_CNT;
    intSave = LOS_IntLock();
    NVIC_DisableIRQ((IRQn_Type)hwiNum);
    LOS_IntRestore(intSave);
    return LOS_OK;
}

UINT32 HalIrqPending(UINT32 hwiNum)
{
    UINT32 intSave;

    if (!HWI_NUM_VALID(hwiNum)) {
        return LOS_ERRNO_HWI_NUM_INVALID;
    }

    hwiNum -= OS_SYS_VECTOR_CNT;
    intSave = LOS_IntLock();
    NVIC_SetPendingIRQ((IRQn_Type)hwiNum);
    LOS_IntRestore(intSave);
    return LOS_OK;
}

UINT32 HalIrqClear(UINT32 hwiNum)
{
    if (!HWI_NUM_VALID(hwiNum)) {
        return LOS_ERRNO_HWI_NUM_INVALID;
    }

    hwiNum -= OS_SYS_VECTOR_CNT;
    NVIC_ClearPendingIRQ((IRQn_Type)hwiNum);
    return LOS_OK;
}

UINT32 HalCurIrqGet(VOID)
{
    g_curIrqNum = __get_IPSR();
    return g_curIrqNum;
}

CHAR *HalIrqVersion(VOID)
{
    return "NVIC";
}

HwiHandleInfo *HalIrqGetHandleForm(HWI_HANDLE_T hwiNum)
{
    if (!HWI_NUM_VALID(hwiNum)) {
        return NULL;
    }

    return &g_hwiForm[hwiNum];
}

STATIC const HwiControllerOps g_nvicOps = {
    .triggerIrq     = HalIrqPending,
    .enableIrq      = HalIrqUnmask,
    .disableIrq     = HalIrqMask,
    .setIrqPriority = HalIrqSetPriority,
    .getCurIrqNum   = HalCurIrqGet,
    .getIrqVersion  = HalIrqVersion,
    .getHandleForm  = HalIrqGetHandleForm,
    .clearIrq       = HalIrqClear,
};

VOID HalIrqInit(VOID)
{
    UINT32 i;

    for (i = OS_SYS_VECTOR_CNT; i < LOSCFG_PLATFORM_HWI_LIMIT; i++) {
        g_hwiVec[i] = (HWI_PROC_FUNC)IrqEntryV7M;
    }
#if (__CORTEX_M == 0x0U)  /* only for Cortex-M0 */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    /* Remap SRAM at 0x00000000 */
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else
    /* Interrupt vector table location */
    SCB->VTOR = (UINT32)g_hwiVec;
#endif
#if (__CORTEX_M >= 0x03U) /* only for Cortex-M3 and above */
    NVIC_SetPriorityGrouping(OS_NVIC_AIRCR_PRIGROUP);
#endif

    /* register interrupt controller's operations */
    OsHwiControllerReg(&g_nvicOps);
    return;
}












//XXX:仅用于试验的PendSV中断触发程序
void triggerPendSVC (void)
{
    MEM8(NVIC_SYSPRI2) = NVIC_PENDSV_PRI;   // 向NVIC_SYSPRI2写NVIC_PENDSV_PRI，设置其为最低优先级
    MEM32(NVIC_INT_CTRL) = NVIC_PENDSVSET;    // 向NVIC_INT_CTRL写NVIC_PENDSVSET，用于PendSV
}

//XXX:从ucos2照抄的systick处理程序
void  OS_CPU_SysTickHandler (void)
{
#if OS_CRITICAL_METHOD == 3u                                    /* Allocate storage for CPU status register             */
    OS_CPU_SR  cpu_sr;
#endif

    OS_ENTER_CRITICAL();
    OSIntEnter();                                               /* Tell uC/OS-II that we are starting an ISR            */
    OS_EXIT_CRITICAL();

    OSTimeTick();                                               /* Call uC/OS-II's OSTimeTick()                         */

    OSIntExit();                                                /* Tell uC/OS-II that we are leaving the ISR            */
}

//XXX：从ucos2照抄的函数
void          OSInit                  (void)
{
#if OS_TASK_CREATE_EXT_EN > 0u
#if defined(OS_TLS_TBL_SIZE) && (OS_TLS_TBL_SIZE > 0u)
    INT8U  err;
#endif
#endif

    OSInitHookBegin();                                           /* Call port specific initialization code   */

    OS_InitMisc();                                               /* Initialize miscellaneous variables       */

    OS_InitRdyList();                                            /* Initialize the Ready List                */

    OS_InitTCBList();                                            /* Initialize the free list of OS_TCBs      */

    OS_InitEventList();                                          /* Initialize the free list of OS_EVENTs    */

#if (OS_FLAG_EN > 0u) && (OS_MAX_FLAGS > 0u)
    OS_FlagInit();                                               /* Initialize the event flag structures     */
#endif

#if (OS_MEM_EN > 0u) && (OS_MAX_MEM_PART > 0u)
    OS_MemInit();                                                /* Initialize the memory manager            */
#endif

#if (OS_Q_EN > 0u) && (OS_MAX_QS > 0u)
    OS_QInit();                                                  /* Initialize the message queue structures  */
#endif

#if OS_TASK_CREATE_EXT_EN > 0u
#if defined(OS_TLS_TBL_SIZE) && (OS_TLS_TBL_SIZE > 0u)
    OS_TLS_Init(&err);                                           /* Initialize TLS, before creating tasks    */
    if (err != OS_ERR_NONE) {
        return;
    }
#endif
#endif

    OS_InitTaskIdle();                                           /* Create the Idle Task                     */
#if OS_TASK_STAT_EN > 0u
    OS_InitTaskStat();                                           /* Create the Statistic Task                */
#endif

#if OS_TMR_EN > 0u
    OSTmr_Init();                                                /* Initialize the Timer Manager             */
#endif

    OSInitHookEnd();                                             /* Call port specific init. code            */

#if OS_DEBUG_EN > 0u
    OSDebugInit();
#endif
}

void          OSIntEnter              (void)
{
    if (OSRunning == OS_TRUE) {
        if (OSIntNesting < 255u) {
            OSIntNesting++;                      /* Increment ISR nesting level                        */
        }
        OS_TRACE_ISR_ENTER();
    }
}

void          OSIntExit               (void)
{
#if OS_CRITICAL_METHOD == 3u                               /* Allocate storage for CPU status register */
    OS_CPU_SR  cpu_sr = 0u;
#endif



    if (OSRunning == OS_TRUE) {
        OS_ENTER_CRITICAL();
        if (OSIntNesting > 0u) {                           /* Prevent OSIntNesting from wrapping       */
            OSIntNesting--;
        }
        if (OSIntNesting == 0u) {                          /* Reschedule only if all ISRs complete ... */
            if (OSLockNesting == 0u) {                     /* ... and not locked.                      */
                OS_SchedNew();
                OSTCBHighRdy = OSTCBPrioTbl[OSPrioHighRdy];
                if (OSPrioHighRdy != OSPrioCur) {          /* No Ctx Sw if current task is highest rdy */
#if OS_TASK_PROFILE_EN > 0u
                    OSTCBHighRdy->OSTCBCtxSwCtr++;         /* Inc. # of context switches to this task  */
#endif
                    OSCtxSwCtr++;                          /* Keep track of the number of ctx switches */

#if OS_TASK_CREATE_EXT_EN > 0u
#if defined(OS_TLS_TBL_SIZE) && (OS_TLS_TBL_SIZE > 0u)
                    OS_TLS_TaskSw();
#endif
#endif
                    OS_TRACE_ISR_EXIT_TO_SCHEDULER();

                    OSIntCtxSw();                          /* Perform interrupt level ctx switch       */
                } else {
                    OS_TRACE_ISR_EXIT();
                }
            } else {
                OS_TRACE_ISR_EXIT();
            }
        } else {
            OS_TRACE_ISR_EXIT();
        }

        OS_EXIT_CRITICAL();
    }
}




void          OSStart                 (void)
{
    if (OSRunning == OS_FALSE) {
        OS_SchedNew();                               /* Find highest priority's task priority number   */
        OSPrioCur     = OSPrioHighRdy;
        OSTCBHighRdy  = OSTCBPrioTbl[OSPrioHighRdy]; /* Point to highest priority task ready to run    */
        OSTCBCur      = OSTCBHighRdy;
        OSStartHighRdy();                            /* Execute target specific code to start task     */
    }
}


void          OSStatInit              (void);
#if OS_TASK_STAT_EN > 0u
void  OSStatInit (void)
{
#if OS_CRITICAL_METHOD == 3u                     /* Allocate storage for CPU status register           */
    OS_CPU_SR  cpu_sr = 0u;
#endif



    OSTimeDly(2u);                               /* Synchronize with clock tick                        */
    OS_ENTER_CRITICAL();
    OSIdleCtr    = 0uL;                          /* Clear idle counter                                 */
    OS_EXIT_CRITICAL();
    OSTimeDly(OS_TICKS_PER_SEC / 10u);           /* Determine MAX. idle counter value for 1/10 second  */
    OS_ENTER_CRITICAL();
    OSIdleCtrMax = OSIdleCtr;                    /* Store maximum idle counter count in 1/10 second    */
    OSStatRdy    = OS_TRUE;
    OS_EXIT_CRITICAL();
}
#endif


//XXX:从ucos2搬运的systick初始化
void  OS_CPU_SysTickInitFreq (INT32U  cpu_freq)
{
    INT32U  cnts;


    cnts = (cpu_freq / (INT32U)OS_TICKS_PER_SEC);               /* Determine nbr SysTick cnts between two OS tick intr. */

    OS_CPU_SysTickInit(cnts);
}


/*
*********************************************************************************************************
*                                         INITIALIZE SYS TICK
*
* Description: Initialize the SysTick using the number of counts between two ticks.
*
* Arguments  : cnts     Number of SysTick counts between two OS tick interrupts.
*
* Note(s)    : 1) This function MUST be called after OSStart() & after processor initialization.
*
*              2) Either OS_CPU_SysTickInitFreq or OS_CPU_SysTickInit() can be called.
*********************************************************************************************************
*/

void  OS_CPU_SysTickInit (INT32U  cnts)
{
    INT32U  prio;
    INT32U  basepri;


                                                                /* Set BASEPRI boundary from the configuration.         */
    basepri               = (INT32U)(CPU_CFG_KA_IPL_BOUNDARY << (8u - CPU_CFG_NVIC_PRIO_BITS));
    OS_CPU_CM_SYST_RVR    = cnts - 1u;                          /* Set Reload register.                                 */

                                                                /* Set SysTick handler prio.                            */
    prio                  =  OS_CPU_CM_SCB_SHPRI3;
    prio                 &=  0x00FFFFFFu;
    prio                 |= (basepri << 24u);
    OS_CPU_CM_SCB_SHPRI3  = prio;

                                                                /* Enable timer.                                        */
    OS_CPU_CM_SYST_CSR   |= OS_CPU_CM_SYST_CSR_CLKSOURCE |
                            OS_CPU_CM_SYST_CSR_ENABLE;
                                                                /* Enable timer interrupt.                              */
    OS_CPU_CM_SYST_CSR   |= OS_CPU_CM_SYST_CSR_TICKINT;
}

//XXX：来自ucos2的任务栈初始化函数
OS_STK *OSTaskStkInit (void (*task)(void *p_arg), void *p_arg, OS_STK *ptos, INT16U opt)
{
    OS_STK *stk;
    INT32U  task_addr;


    opt       = opt;                             /* 'opt' is not used, prevent warning                 */
    stk       = ptos;                            /* Load stack pointer                                 */
    task_addr = (INT32U)task & ~1u;              /* Mask off lower bit in case task is thumb mode      */
    *(stk)    = (INT32U)task_addr;               /* Entry Point                                        */
    *(--stk)  = (INT32U)OS_TaskReturn;           /* R14 (LR)                                           */
    *(--stk)  = (INT32U)0x12121212uL;            /* R12                                                */
    *(--stk)  = (INT32U)0x11111111uL;            /* R11                                                */
    *(--stk)  = (INT32U)0x10101010uL;            /* R10                                                */
    *(--stk)  = (INT32U)0x09090909uL;            /* R9                                                 */
    *(--stk)  = (INT32U)0x08080808uL;            /* R8                                                 */
    *(--stk)  = (INT32U)0x07070707uL;            /* R7                                                 */
    *(--stk)  = (INT32U)0x06060606uL;            /* R6                                                 */
    *(--stk)  = (INT32U)0x05050505uL;            /* R5                                                 */
    *(--stk)  = (INT32U)0x04040404uL;            /* R4                                                 */
    *(--stk)  = (INT32U)0x03030303uL;            /* R3                                                 */
    *(--stk)  = (INT32U)0x02020202uL;            /* R2                                                 */
    *(--stk)  = (INT32U)0x01010101uL;            /* R1                                                 */
    *(--stk)  = (INT32U)p_arg;                   /* R0 : argument                                      */
    if ((INT32U)task & 0x01u) {                  /* See if task runs in Thumb or ARM mode              */
        *(--stk) = (INT32U)ARM_SVC_MODE_THUMB;   /* CPSR  (Enable IRQ and FIQ interrupts, THUMB-mode)  */
    } else {
        *(--stk) = (INT32U)ARM_SVC_MODE_ARM;     /* CPSR  (Enable IRQ and FIQ interrupts, ARM-mode)    */
    }

    return (stk);
}











//XXX:从FreeRTOS照抄的调度函数
BaseType_t xPortStartScheduler(void)
{
    /* configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.
     * See https://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
    configASSERT(configMAX_SYSCALL_INTERRUPT_PRIORITY);

    /* This port can be used on all revisions of the Cortex-M7 core other than
     * the r0p1 parts.  r0p1 parts should use the port from the
     * /source/portable/GCC/ARM_CM7/r0p1 directory. */
    configASSERT(portCPUID != portCORTEX_M7_r0p1_ID);
    configASSERT(portCPUID != portCORTEX_M7_r0p0_ID);

#if ( configASSERT_DEFINED == 1 )
    {
        volatile uint32_t ulOriginalPriority;
        volatile uint8_t* const pucFirstUserPriorityRegister = (uint8_t*)(portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER);
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
        configASSERT(ucMaxPriorityValue == (configKERNEL_INTERRUPT_PRIORITY & ucMaxPriorityValue));

        /* Use the same mask on the maximum system call priority. */
        ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

        /* Calculate the maximum acceptable priority group value for the number
         * of bits read back. */
        ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;

        while ((ucMaxPriorityValue & portTOP_BIT_OF_BYTE) == portTOP_BIT_OF_BYTE)
        {
            ulMaxPRIGROUPValue--;
            ucMaxPriorityValue <<= (uint8_t)0x01;
        }

#ifdef __NVIC_PRIO_BITS
        {
            /* Check the CMSIS configuration that defines the number of
             * priority bits matches the number of priority bits actually queried
             * from the hardware. */
            configASSERT((portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue) == __NVIC_PRIO_BITS);
        }
#endif

#ifdef configPRIO_BITS
        {
            /* Check the FreeRTOS configuration that defines the number of
             * priority bits matches the number of priority bits actually queried
             * from the hardware. */
            configASSERT((portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue) == configPRIO_BITS);
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
    *(portFPCCR) |= portASPEN_AND_LSPEN_BITS;

    /* Start the first task. */
    prvStartFirstTask();

    /* Should not get here! */
    return 0;
}