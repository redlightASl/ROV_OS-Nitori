#include "Thread.h"
#include "System.h"


//XXX:照抄rtt的main线程机制
void rt_application_init(void);
void rt_hw_board_init(void);
int rtthread_startup(void);

#ifdef __ARMCC_VERSION
extern int $Super$$main(void);
/* re-define main function */
int $Sub$$main(void)
{
    rtthread_startup();
    return 0;
}
#elif defined(__ICCARM__)
extern int main(void);
/* __low_level_init will auto called by IAR cstartup */
extern void __iar_data_init3(void);
int __low_level_init(void)
{
    // call IAR table copy function.
    __iar_data_init3();
    rtthread_startup();
    return 0;
}
#elif defined(__GNUC__)
/* Add -eentry to arm-none-eabi-gcc argument */
int entry(void)
{
    rtthread_startup();
    return 0;
}
#endif

#ifndef RT_USING_HEAP
/* if there is not enable heap, we should use static thread and stack. */
ALIGN(8)
static rt_uint8_t main_stack[RT_MAIN_THREAD_STACK_SIZE];
struct rt_thread main_thread;
#endif /* RT_USING_HEAP */

/* 进入main线程 */
void main_thread_entry(void *parameter)
{
    extern int main(void);

    /* 初始化外设驱动组件 */

    /* 进入main线程 */
#ifdef __ARMCC_VERSION
    {
        extern int $Super$$main(void);
        $Super$$main(); /* for ARMCC. */
    }
#elif defined(__ICCARM__) || defined(__GNUC__) || defined(__TASKING__)
    main();
#endif
}


/* 创建用户main线程 */
void rt_application_init(void)
{
    rt_thread_t tid;

#ifdef RT_USING_HEAP
    tid = rt_thread_create("main", main_thread_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, RT_MAIN_THREAD_PRIORITY, 20);
    RT_ASSERT(tid != RT_NULL);
#else
    rt_err_t result;

    tid = &main_thread;
    result = rt_thread_init(tid, "main", main_thread_entry, RT_NULL,
                            main_stack, sizeof(main_stack), RT_MAIN_THREAD_PRIORITY, 20);
    RT_ASSERT(result == RT_EOK);

    /* if not define RT_USING_HEAP, using to eliminate the warning */
    (void)result;
#endif /* RT_USING_HEAP */

    rt_thread_startup(tid);
}




int system_init(void)
{
    /* 关闭中断 */
    rt_hw_interrupt_disable();

    /* 初始化硬件 */
    rt_hw_board_init();

    /* 串口输出版本信息和硬件数据 */
    rt_show_version();

    /* 初始化系统时钟 */
    rt_system_timer_init();

    /* 初始化线程调度器 */
    rt_system_scheduler_init();

    /* 信号量初始化 */
    rt_system_signal_init();

    /* 创建用户main线程 */
    rt_application_init();

    /* 创建定时器线程 */
    rt_system_timer_thread_init();

    /* 创建空闲线程 */
    rt_thread_idle_init();

    /* 启动线程调度器 */
    rt_system_scheduler_start();

    /* 不会运行至此 */
    return 0;
}







/*
 * Description : Tick interruption handler
 */
LITE_OS_SEC_TEXT VOID OsTickHandler(VOID)
{
    UINT32 intSave;

    TICK_LOCK(intSave);
    g_tickCount[ArchCurrCpuid()]++;
    TICK_UNLOCK(intSave);

#ifdef LOSCFG_KERNEL_TICKLESS
    if (g_tickWakeupHook != NULL) {
        g_tickWakeupHook(LOS_TICK_INT_FLAG);
    }
#endif

#ifdef LOSCFG_BASE_CORE_TIMESLICE
    OsTimesliceCheck();
#endif

    OsTaskScan(); /* task timeout scan */

#ifdef LOSCFG_BASE_CORE_SWTMR
    OsSwtmrScan();
#endif
}

LITE_OS_SEC_TEXT_INIT UINT32 OsTickInit(UINT32 systemClock, UINT32 tickPerSecond)
{
    if ((systemClock == 0) ||
        (tickPerSecond == 0) ||
        (tickPerSecond > systemClock)) {
        return LOS_ERRNO_TICK_CFG_INVALID;
    }
    HalClockInit();

    return LOS_OK;
}

LITE_OS_SEC_TEXT_INIT VOID OsTickStart(VOID)
{
    HalClockStart();
}

LITE_OS_SEC_TEXT_MINOR UINT64 LOS_TickCountGet(VOID)
{
    UINT32 intSave;
    UINT64 tick;

    /*
     * use core0's tick as system's timeline,
     * the tick needs to be atomic.
     */
    TICK_LOCK(intSave);
    tick = g_tickCount[0];
    TICK_UNLOCK(intSave);

    return tick;
}

LITE_OS_SEC_TEXT_MINOR UINT32 LOS_CyclePerTickGet(VOID)
{
    return g_sysClock / KERNEL_TICK_PER_SECOND;
}

LITE_OS_SEC_TEXT_MINOR VOID LOS_GetCpuCycle(UINT32 *highCnt, UINT32 *lowCnt)
{
    UINT64 cycle;

    if ((highCnt == NULL) || (lowCnt == NULL)) {
        return;
    }
    cycle = HalClockGetCycles();

    /* get the high 32 bits */
    *highCnt = (UINT32)(cycle >> 32);
    /* get the low 32 bits */
    *lowCnt = (UINT32)(cycle & 0xFFFFFFFFULL);
}

LITE_OS_SEC_TEXT_MINOR UINT64 LOS_CurrNanosec(VOID)
{
    UINT64 nanos;

    nanos = HalClockGetCycles() * (OS_SYS_NS_PER_SECOND / OS_SYS_NS_PER_MS) / (g_sysClock / OS_SYS_NS_PER_MS);
    return nanos;
}

LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MS2Tick(UINT32 millisec)
{
    UINT64 delaySec;

    if (millisec == UINT32_MAX) {
        return UINT32_MAX;
    }

    delaySec = (UINT64)millisec * KERNEL_TICK_PER_SECOND;
    return (UINT32)((delaySec + OS_SYS_MS_PER_SECOND - 1) / OS_SYS_MS_PER_SECOND);
}

LITE_OS_SEC_TEXT_MINOR UINT32 LOS_Tick2MS(UINT32 tick)
{
    return (UINT32)(((UINT64)tick * OS_SYS_MS_PER_SECOND) / KERNEL_TICK_PER_SECOND);
}

LITE_OS_SEC_TEXT_MINOR VOID LOS_Udelay(UINT32 usecs)
{
    HalDelayUs(usecs);
}

LITE_OS_SEC_TEXT_MINOR VOID LOS_Mdelay(UINT32 msecs)
{
    UINT32 delayUs = (UINT32_MAX / OS_SYS_US_PER_MS) * OS_SYS_US_PER_MS;

    while (msecs > UINT32_MAX / OS_SYS_US_PER_MS) {
        HalDelayUs(delayUs);
        msecs -= (UINT32_MAX / OS_SYS_US_PER_MS);
    }
    HalDelayUs(msecs * OS_SYS_US_PER_MS);
}

VOID OsSchedResched(VOID)
{
    LosTaskCB *runTask = NULL;
    LosTaskCB *newTask = NULL;

    LOS_ASSERT(LOS_SpinHeld(&g_taskSpin));

    if (!OsPreemptableInSched()) {
        return;
    }

    runTask = OsCurrTaskGet();
    newTask = OsGetTopTask();

    /* always be able to get one task */
    LOS_ASSERT(newTask != NULL);

    newTask->taskStatus &= ~OS_TASK_STATUS_READY;

    if (runTask == newTask) {
        return;
    }

    runTask->taskStatus &= ~OS_TASK_STATUS_RUNNING;
    newTask->taskStatus |= OS_TASK_STATUS_RUNNING;

#ifdef LOSCFG_KERNEL_SMP
    /* mask new running task's owner processor */
    runTask->currCpu = OS_TASK_INVALID_CPUID;
    newTask->currCpu = ArchCurrCpuid();
#endif

    OsTaskTimeUpdateHook(runTask->taskId, LOS_TickCountGet());

#ifdef LOSCFG_KERNEL_CPUP
    OsTaskCycleEndStart(newTask);
#endif

#ifdef LOSCFG_BASE_CORE_TSK_MONITOR
    OsTaskSwitchCheck(runTask, newTask);
#endif

    LOS_TRACE(TASK_SWITCH, newTask->taskId, runTask->priority, runTask->taskStatus, newTask->priority,
        newTask->taskStatus);

#ifdef LOSCFG_DEBUG_SCHED_STATISTICS
    OsSchedStatistics(runTask, newTask);
#endif

#ifdef LOSCFG_BASE_CORE_TIMESLICE
    if (newTask->timeSlice == 0) {
        newTask->timeSlice = KERNEL_TIMESLICE_TIMEOUT;
    }
#endif

    OsCurrTaskSet((VOID*)newTask);

    /* do the task context switch */
    OsTaskSchedule(newTask, runTask);
}