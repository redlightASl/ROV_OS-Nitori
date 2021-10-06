#include "Thread.h"

//DEBUG：这个函数预计会转移到Port.h
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


//XXX:仅用于试验的PendSV中断触发程序
void triggerPendSVC (void)
{
    MEM8(NVIC_SYSPRI2) = NVIC_PENDSV_PRI;   // 向NVIC_SYSPRI2写NVIC_PENDSV_PRI，设置其为最低优先级
    MEM32(NVIC_INT_CTRL) = NVIC_PENDSVSET;    // 向NVIC_INT_CTRL写NVIC_PENDSVSET，用于PendSV
}
