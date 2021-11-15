#include <Interrupt.h>

//DONE:应用程序临界区处理函数
/**
 * @brief 应用程序进入临界区
 */
void rov_enter_critical(void)
{
    rov_BaseType level;
    level = DISABLE_INTERRUPTS();
    INTERRUPT_NEST++;
    ENABLE_INTERRUPTS(level);
}

/**
 * @brief 应用程序退出临界区
 */
void rov_exit_critical(void)
{
    rov_BaseType level;
    level = DISABLE_INTERRUPTS();
    INTERRUPT_NEST--;
    ENABLE_INTERRUPTS(level);
}

/**
 * @brief 获取当前中断层数
 * @return vu8 嵌套中断层数
 * @note 应用程序使用该函数了解当前是否处于中断状态（临界区）
 */
vu8 rov_critical_level(void)
{
    vu8 ret;
    rov_BaseType level;
    level = DISABLE_INTERRUPTS();
    ret = INTERRUPT_NEST;
    ENABLE_INTERRUPTS(level);
    return ret;
}

//TODO:系统时钟处理
//需要使用Port.h的接口




rt_tick_t rt_tick_get(void)
{
    /* return the global tick */
    return rt_tick;
}

void rt_tick_set(rt_tick_t tick)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    rt_tick = tick;
    rt_hw_interrupt_enable(level);
}

rt_tick_t rt_tick_from_millisecond(rt_int32_t ms)
{
    rt_tick_t tick;

    if (ms < 0)
    {
        tick = (rt_tick_t)RT_WAITING_FOREVER;
    }
    else
    {
        tick = RT_TICK_PER_SECOND * (ms / 1000);
        tick += (RT_TICK_PER_SECOND * (ms % 1000) + 999) / 1000;
    }

    /* return the calculated tick */
    return tick;
}

RT_WEAK rt_tick_t rt_tick_get_millisecond(void)
{
#if 1000 % RT_TICK_PER_SECOND == 0u
    return rt_tick_get() * (1000u / RT_TICK_PER_SECOND);
#else
    #warning "rt-thread cannot provide a correct 1ms-based tick any longer,\
    please redefine this function in another file by using a high-precision hard-timer."
        return 0;
#endif /* 1000 % RT_TICK_PER_SECOND == 0u */
}


